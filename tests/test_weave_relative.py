# -*- coding: utf-8 -*-
"""T3b:S 型時間同步的**參考量**值層級鎖(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
`_weave_theta_of` 原本用**絕對**參考量:

    e_t = (dive_T − now) − t_go

它量的是「離協商出來的時刻表還差多少」,而不是「比隊友早多少」。
⇒ 指令的**量級**由 dive_T 談得準不準決定。實測(runs/artifacts 全套
143 個架次 / 70441 個 wait 拍):

  * 42/143(**29%**)的架次中位 |e_t| > 60 s;
  * 這些架次的解析角 cos θ = t_go/(t_go+e_t) 直接把指令推到夾限 ——
    RA1 u2 與 SB3 u2 **整段 100% 飽和**、LAT1 u1 91%;
  * 全隊一起釘在 45° = 每一架把路徑拉長同樣的最大倍率
    ⇒ **同步不到任何東西**,而契約行仍印出「已啟用」。

⇒ `weave_relative=1` 改成與 ring_pace 同一個藥方:

    e_t = max(eta_全隊) − eta_自己       ← eta 由**實測地速**估

量級由**機間差**決定 ⇒ 不再被 dive_T 的談判誤差污染;恆 ≥ 0、
最晚那一架恰好 0(不擺)。

⭐ 文獻對位(2026-08-23 經 LM 取逐字引述):橫向偏置是**單向**的 ——
   負偏置只能把前置角壓回 0(直線 PN),那就是最短時間,再負也縮不短
   ⇒ 共識點必須取 max(可達下界),取加權平均會讓較慢的機收到不可達指令。

⚠️⚠️ **本檔第一版的前提是錯的。** 它寫「e_t 恆為負 ⇒ 死區恆成立 ⇒ 蛇形
   一次都不會擺,等同 ring_pace 註解記載的 RRB 8 趟白跑」。那是把 ring_pace
   的軼事外推到 weave,而同一批飛行資料推翻了它:**44.41%** 的 wait 拍超過
   死區,兩架 e_t 差值與實測 eta 差的相關 **r = 0.942**(方向是對的)。
   失效是**飽和**,不是不執行 —— 修法一樣,理由不一樣,而理由寫錯會讓
   下一個人拿它去推別的結論。

═══════════════════════════════════════════════════════════════════════════
 ⭐ 為什麼一條源碼字串比對都沒有
═══════════════════════════════════════════════════════════════════════════
源碼層鎖驗的是「那一行長什麼樣」,改上游/下游/搬進 helper 全部免疫
([[value-locks-beat-source-locks]]:值層級一次關掉 8 個,舊 38 條零反應)。
本檔一律**餵一組數字進去、對送出來的角度下斷言**。
唯一走 AST 的是「把受測方法從檔案原始碼綁成 mixin」——
與 `tests/test_nz_closed_loop.py`、`tests/test_mpng_value_locks.py` 同一套手法。
"""
import ast
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


def _mixin(name):
    """把受測方法從檔案原始碼綁成一個孤立的類別。

    ⚠️ 模組層只搬進這個方法真正碰得到的東西(`math`、`GRAV`、`dist_m`)。
       其餘依賴(`_geom` / `ring_of` / `_tgo_of`)一律由樁提供 ——
       本檔測的是**參考量的選擇**,不是幾何計算。
    """
    ns = {"math": math}
    #: ⚠️ 模組層常數與純函式**一律自動搬過來**,不逐個列名 —— 逐個列名的寫法
    #: 會在每次重構時丟一個看起來像「測試壞了」的 NameError,而那正是讓人
    #: 把鎖停掉的原因(與 test_nz_closed_loop 同一個理由)。
    for n in TREE.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(
                n.targets[0], ast.Name):
            try:
                ns[n.targets[0].id] = ast.literal_eval(n.value)
            except Exception:                                 # noqa: BLE001
                pass
    for n in TREE.body:
        if isinstance(n, ast.FunctionDef):
            try:
                exec(ast.get_source_segment(SRC, n), ns)      # noqa: S102
            except Exception:                                 # noqa: BLE001
                pass
    #: ⚠️⚠️ 這一條是實打實抓到過東西的:被測碼原本寫 `GRAV`,而模組層
    #: **只有 G0** —— GRAV 是 `_pn_cmd` 裡的區域變數 ⇒ 那一行是 NameError,
    #: 只是被上游的死區 return 遮住了。所以名字解析必須是**斷言**,
    #: 不能靠「反正 exec 得起來」。
    assert "G0" in ns, "tot_node.py 找不到模組層的 G0"
    assert "dist_m" in ns, "tot_node.py 找不到 dist_m()"
    body = "\n".join(
        "    " + ln for ln in ast.get_source_segment(SRC, _func(name)).splitlines())
    exec("class _M:\n" + body, ns)                            # noqa: S102
    return ns["_M"]


_W = _mixin("_weave_theta_of")

GATE_FRAC = 1.1
RING = 6000.0
GATE = RING * GATE_FRAC


class Node(_W):
    """⚠️ 樁只提供依賴,不重寫任何被測邏輯。

    `_tgo_of` 刻意做成「**永遠低估**」—— 那正是實機預測器的行為
    (27/27 單邊為正、中位 +18.7 s),也正是絕對式失效的原因。
    """

    #: 兩架都在目標正東方,uav1 近、uav2 遠。緯度 0 讓經度換算單純。
    T = (0.0, 0.0)

    def __init__(self, *, relative, dive_T=600.0, gs=None, dists=None,
                 theta_max=45.0, period=120.0, deadband=0.5, tgo_bias=-18.7):
        self.weave_time_sync = True
        self.weave_relative = relative
        self.weave_theta_max = theta_max
        self.weave_period = period
        self.weave_deadband_s = deadband
        self.dive_T = dive_T
        self._tgo_bias = tgo_bias
        self.ring_gate_frac = GATE_FRAC
        self._ring = RING
        self.e_t = {}
        self.e_t_rel = {}
        self.weave_theta = {}
        self.gs = dict(gs or {1: 170.0, 2: 170.0})
        d = dict(dists or {1: 12000.0, 2: 20000.0})
        #: 把「離目標多遠」換算成經度(緯度 0 ⇒ 1 度經度 = 111320 m)。
        self.pos = {q: (0.0, dq / 111320.0, 3100.0) for q, dq in d.items()}
        self._d = d

    def ring_of(self, i):
        return self._ring

    def _geom(self, i):
        g = [0.0] * 10
        g[9] = self.gs[i]
        return g

    def _tgo_of(self, i, d):
        """真實的 t_go 加上預測器的偏誤(負 = 低估 ⇒ 飛機顯得遲到)。"""
        return max(1.0, d / max(self.gs[i], 1.0) + self._tgo_bias)


def theta(n, i):
    return n._weave_theta_of(i, n._d[i], 0.0)


def eta_of(n, i):
    """本檔**獨立**算一遍 eta,不讀被測碼。"""
    return (n._d[i] - GATE) / n.gs[i]


# ══════════════════════════════════════════════════════════════════════════
class TestL1TheAbsoluteReferenceSaturates:
    """⭐⭐ 先把失效**量化**,再證明修法有效。

    ⚠️ 失效不是「不會擺」(那個說法已被 44.41% 的實測拍數推翻),
       是「擺的量級由時刻表誤差決定 ⇒ 整段釘在夾限」。
    """

    #: 實測的形狀:29% 的架次中位 |e_t| > 60 s。取 145 s(RA1/SB3 的量級)。
    OFF_BY = 145.0

    def test_a_mis_negotiated_timetable_pins_the_command_at_the_clamp(self):
        """⚠️⚠️ 這一條**故意**斷言壞行為 —— 它是修法的存在理由。

        uav1 的預測 t_go = 12000/170 − 18.7 = 51.9 s。時刻表訂在
        51.9 + 145 = 196.9 s ⇒ e_t = 145 s ⇒ cos θ = 51.9/196.9 = 0.264
        ⇒ 解析角 74.7°,遠超過任何夾限 ⇒ **兩架都釘在夾限**。
        兩架拉長同樣的最大倍率 ⇒ 同步不到任何東西。
        """
        n = Node(relative=False, dive_T=51.9 + self.OFF_BY)
        cap = min(n.weave_theta_max,
                  math.degrees(9.80665 * math.tan(math.radians(45.0)) / 170.0)
                  * n.weave_period / 8.0)
        th1, th2 = theta(n, 1), theta(n, 2)
        assert th1 == pytest.approx(cap), "uav1 應該被夾限咬住"
        assert th2 == pytest.approx(cap), "uav2 也被咬住 ⇒ 兩架完全一樣"
        assert th1 == th2, (
            "⚠️⚠️ 兩架收到**完全相同**的角度 ⇒ 路徑拉長同樣倍率 ⇒ "
            "機間差原封不動 ⇒ 蛇形對同步零貢獻,而 log 一切正常")

    def test_the_relative_reference_gives_the_two_ships_different_angles(self):
        """⭐ 同一組數字,只換參考量 ⇒ 兩架收到**不同**的角度,
        而且最晚那一架恰好 0 —— 那才是同步。"""
        n = Node(relative=True, dive_T=51.9 + self.OFF_BY)
        th1, th2 = theta(n, 1), theta(n, 2)
        assert th1 > 0.0, "早到的那一架必須擺"
        assert th2 == 0.0, "最晚抵達的那一架不該擺(它就是基準)"
        assert th1 != th2

    def test_the_absolute_command_is_driven_by_the_timetable_error(self):
        """⭐⭐ 判別式:把時刻表誤差從 0 掃到 145 s,**幾何完全不動**。

        絕對式的角度會從 0 一路長到夾限(它在反應一個與機間差無關的量);
        相對式必須**逐位元不動**。
        """
        got_abs, got_rel = [], []
        for off in (0.0, 5.0, 40.0, 145.0):
            got_abs.append(theta(Node(relative=False, dive_T=51.9 + off), 1))
            got_rel.append(theta(Node(relative=True, dive_T=51.9 + off), 1))
        assert got_abs[0] == 0.0 and got_abs[-1] > 40.0, (
            f"絕對式應該隨時刻表誤差變化,實得 {got_abs}")
        assert len(set(got_rel)) == 1, (
            f"⚠️⚠️ 相對式被 dive_T 影響了 ⇒ 污染源沒切乾淨,實得 {got_rel}")

    def test_the_relative_error_is_never_negative(self):
        """⭐ 相對式的定義保證 e_t ≥ 0 ⇒ 不存在「要求飛機縮短路徑」這件事。

        文獻對位:橫向偏置是單向的,負值只能把前置角壓回 0(直線 PN),
        再負也縮不短 ⇒ 共識點取 max 是唯一每架都可行的選擇。
        """
        n = Node(relative=True)
        for i in (1, 2):
            theta(n, i)
            assert n.e_t_rel[i] >= 0.0

    def test_the_two_error_channels_are_not_the_same_slot(self):
        """⚠️⚠️ 相對值**絕不可以**寫回 self.e_t —— 那是 `_observe_timing()`
        的純觀測通道,CSV 欄 early_by_s 就是它。兩個算式塞同一格時,
        那一欄會變成「看哪個寫入者最後跑」的混合值,而且完全靜默。
        (改成相對式之前兩者算式相同所以無害,改完就有害了。)"""
        n = Node(relative=True)
        theta(n, 1)
        assert 1 in n.e_t_rel, "相對式必須留下自己的觀測量"
        assert 1 not in n.e_t, (
            "⚠️⚠️ 相對式污染了絕對式的觀測通道 ⇒ early_by_s 那一欄變成兩個量的混合")


class TestL2TheRelativeReferenceDoesNotTouchTheSchedule:
    def test_it_weaves_even_when_the_timetable_is_not_locked_yet(self):
        """⚠️ 第一版兩個分支共用 `dive_T is None` 守衛 ⇒ 相對式在時刻表
        還沒鎖定時也不擺,而它根本不需要時刻表。"""
        n = Node(relative=True, dive_T=None)
        assert theta(n, 1) > 0.0

    @pytest.mark.parametrize("dive_T", [10.0, 600.0, 99999.0, None])
    def test_the_commanded_angle_is_independent_of_dive_t(self, dive_T):
        """值層級鎖:dive_T 動四個數量級(含 None),角度必須逐位元相同。"""
        assert theta(Node(relative=True, dive_T=dive_T), 1) == theta(
            Node(relative=True, dive_T=600.0), 1)

    @pytest.mark.parametrize("bias", [-40.0, -18.7, 0.0, +40.0])
    def test_the_commanded_angle_is_independent_of_the_predictor_bias(self, bias):
        """同理:預測器偏誤動 ±40 s,相對式必須完全不受影響。"""
        assert theta(Node(relative=True, tgo_bias=bias), 1) == theta(
            Node(relative=True, tgo_bias=0.0), 1)


class TestL3TheArithmeticIsTheOneWeIntend:
    def test_a_small_error_gives_the_closed_form_angle(self):
        """cosθ = t_go/(t_go+e_t)。用**獨立**算一遍的值比對,不讀被測碼。

        ⭐ 需求小時角度必須跟著小 —— 否則是「常數輸出」偽裝成閉迴路。
        """
        n = Node(relative=True, dists={1: 12000.0, 2: 12400.0})
        want = math.degrees(math.acos(eta_of(n, 1) / eta_of(n, 2)))
        got = theta(n, 1)
        assert got == pytest.approx(want, rel=1e-9)
        # ⚠️ 必須確認這個角度是**解析式**給的,不是被夾限咬出來的 ——
        #    否則「角度對得上閉式解」這句話是假的(常數輸出也會通過)。
        omega = math.degrees(9.80665 * math.tan(math.radians(45.0)) / 170.0)
        assert got < n.weave_theta_max and got < omega * n.weave_period / 8.0

    def test_the_angle_grows_monotonically_with_the_time_error(self):
        prev = -1.0
        for d2 in (12100.0, 12400.0, 13000.0, 14000.0):
            n = Node(relative=True, dists={1: 12000.0, 2: d2}, theta_max=90.0)
            th = theta(n, 1)
            assert th > prev, "時間差變大而角度沒跟著變大"
            prev = th

    def test_the_stretch_actually_absorbs_the_error(self):
        """⭐⭐ 端到端的意義檢查:1/cosθ 拉長的航程,除以速度,
        必須等於要吸收的時間差。這一條才是「蛇形有沒有用」的定義。"""
        n = Node(relative=True, dists={1: 12000.0, 2: 12400.0})
        th = math.radians(theta(n, 1))
        assert th > 0.0
        extra_s = eta_of(n, 1) * (1.0 / math.cos(th) - 1.0)
        want = eta_of(n, 2) - eta_of(n, 1)
        assert extra_s == pytest.approx(want, rel=1e-9)


class TestL4TheDeadbandAndTheClamps:
    def test_a_difference_inside_the_deadband_does_not_weave(self):
        """兩架幾乎同時 ⇒ e_t < deadband ⇒ 不擺(避免擺盪)。"""
        n = Node(relative=True, dists={1: 12000.0, 2: 12040.0})
        assert theta(n, 1) == 0.0

    def test_the_reversal_rate_clamp_binds_at_the_foam_era_period(self):
        """⚠️⚠️ 週期 22 s 是泡棉時代的值。這一條把「開關開了但吸不到時間」
        變成一個**數字**:同一組需求下 22 s 只給 9.8°,120 s 給 53.7°。"""
        # ⚠️ 地速刻意用 157 —— 那正是不變式
        #    `_inv_weave_period_can_actually_absorb_time` 的 docstring 算 9.8/53.7
        #    用的速度。這樣這一條同時也在驗那份文件裡的數字。
        #    (第一版用樁的預設 170,量到 9.09/49.58,那不是文件裡的數。)
        gs = {1: 157.0, 2: 157.0}
        slow = theta(Node(relative=True, period=22.0, theta_max=90.0, gs=gs), 1)
        fast = theta(Node(relative=True, period=120.0, theta_max=90.0, gs=gs), 1)
        assert slow == pytest.approx(9.8, abs=0.1)
        assert fast == pytest.approx(53.7, abs=0.1)
        assert 1.0 / math.cos(math.radians(slow)) < 1.02, (
            "22 s 週期下路徑只拉長 <2% ⇒ 吸收力小於 1 s,而我們要吸 11–14 s")

    def test_the_user_ceiling_binds(self):
        assert theta(Node(relative=True, theta_max=12.0), 1) == pytest.approx(12.0)


class TestL5NoFalseReference:
    """⚠️ 取不到隊友資料時**必須不擺**,而不是拿一個假的基準去擺。"""

    def test_a_single_ship_does_not_weave(self):
        """⚠️ 攔下它的是**死區**不是專門的守衛:單機時 max(eta)==eta[i]
        ⇒ e_t 恰好 0。2026-08-23 的刪除實驗證實加一條 `len(eta) < 2`
        不會讓任何測試變紅 ⇒ 那條已移除(擋不到東西的守衛比沒有更糟)。"""
        n = Node(relative=True, gs={1: 170.0}, dists={1: 12000.0})
        assert theta(n, 1) == 0.0
        assert n.e_t_rel[1] == 0.0

    def test_a_ship_without_its_own_groundspeed_does_not_weave(self):
        """本機地速還沒到 ⇒ 沒有基準 ⇒ 不擺(而不是拿隊友的當自己的)。"""
        n = Node(relative=True, gs={1: 0.0, 2: 170.0})
        assert theta(n, 1) == 0.0

    def test_a_stationary_teammate_is_not_used_as_the_reference(self):
        """地速 ~0 的隊友會給出無限大的 eta ⇒ 本機被要求擺到夾限。
        那是**資料缺失**不是時間差 ⇒ 必須排除。"""
        n = Node(relative=True, gs={1: 170.0, 2: 0.0})
        assert theta(n, 1) == 0.0, (
            "⚠️⚠️ 拿地速 0 的隊友當基準 ⇒ eta=∞ ⇒ 本機一路擺到夾限,"
            "而那架其實只是還沒起飛/資料還沒到")

    def test_a_ship_already_past_the_gate_does_not_pull_the_reference_negative(self):
        """已經越過放行閘門的機體 eta 夾在 0 ⇒ 它不會變成「最晚者」,
        也不會讓別人算出負的 e_t。"""
        n = Node(relative=True, dists={1: 3000.0, 2: 20000.0})
        theta(n, 1)
        assert n.e_t_rel[1] == pytest.approx(eta_of(n, 2) - 0.0, rel=1e-9)

    def test_the_switch_off_is_bit_exact_zero(self):
        n = Node(relative=True)
        n.weave_time_sync = False
        assert theta(n, 1) == 0.0
