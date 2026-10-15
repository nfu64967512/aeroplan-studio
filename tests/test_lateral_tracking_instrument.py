# -*- coding: utf-8 -*-
"""①②③ 橫向實得側 / 蛇形窗口 / 配速殘差的值層級鎖(2026-08-24)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
[[gamma-tracking-deficit]] 量到實機 γ 追蹤增益 **0.063**、模型 1.385 ——
差 22 倍,遲滯 8 s、26/32 觸頂。那是**俯仰**通道,而它是所有模擬器建不出
我們失效母體的單一根因。

橫向有沒有同樣的缺陷,**一層都沒有人量過**。而 path stretching(蛇形 /
BPNG / MPNG 的時間項)整條路都押在「橫向拉得出彎」上。⇒ 這是先決條件。

要量的是**兩層不同的東西**,而混淆它們會得到一個看起來合理的錯答案:

    roll_meas_deg    vs 既有的 roll_cmd_deg   → 坡度**拉不拉得出來**
    chi_dot_meas_dps vs chi_dot_kin_dps       → 坡度**轉不轉得出航向**

γ 的 0.063 是第一層那一類。兩層只要有一層塌掉,偏置項下再多指令都彎不出去。

═══════════════════════════════════════════════════════════════════════════
 ⚠️ 為什麼「實得側」以前刻意不做,而這次非做不可
═══════════════════════════════════════════════════════════════════════════
tot_node.py:3600 那段寫得很清楚:實得側要對遙測速度微分,而「dt 來源正是
本專案重複踩過的坑」([[impact-speed-reads-groundspeed]]:dt=0 讓 vz 釘 0,
12/16 架次的觸地總速靜默退化成地速),離線重建的尾巴是微分窗假象
(>3g 佔比隨半窗 1→16 從 2.01% 掉到 0.71%),結論是「**要它就得線上量**」。

⇒ 本檔逐條鎖住那句話點名的每一個陷阱:

  ① dt 取自 **state_t 的推進**,不是牆鐘。
     [[leader-state-freeze]]:長機位姿凍結但仍以 10 Hz 廣播,「停發有告警,
     凍結沒有」。拿牆鐘微分凍結的狀態會得到 χ̇ = 0,讀起來像「這架沒在
     轉彎」—— 而那個誤讀的方向剛好會把**好機體判成拉不動**,
     也就是會讓我們做出跟事實相反的決定。
  ② χ 必須**去纏繞**。+179° → −179° 是轉了 2°,不是 −358°。
  ③ **dt 本身也要記**,否則分不出「真的轉得快」與「窗太短放大雜訊」。

═══════════════════════════════════════════════════════════════════════════
 ⭐ 一條源碼字串比對都沒有
═══════════════════════════════════════════════════════════════════════════
源碼層鎖驗「那一行長什麼樣」,改上游/下游/搬進 helper 全免疫
([[value-locks-beat-source-locks]])。本檔一律**餵數字、對回傳值下斷言**。
唯一走 AST 的是把受測方法綁成 mixin —— 與 test_weave_relative.py 同一套。
"""
import ast
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

G0 = 9.80665


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError("tot_node.py 找不到 %s()" % name)


def _mixin(*names):
    """把受測方法從檔案原始碼綁成一個孤立的類別。

    ⚠️ 模組層常數與純函式**一律自動搬過來**,不逐個列名 —— 逐個列名會在
       每次重構時丟一個看起來像「測試壞了」的 NameError,而那正是讓人把鎖
       停掉的原因。
    """
    ns = {"math": math}
    for n in TREE.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(
                n.targets[0], ast.Name):
            try:
                ns[n.targets[0].id] = ast.literal_eval(n.value)
            except Exception:                                  # noqa: BLE001
                pass
    for n in TREE.body:
        if isinstance(n, ast.FunctionDef):
            try:
                exec(ast.get_source_segment(SRC, n), ns)       # noqa: S102
            except Exception:                                  # noqa: BLE001
                pass
    # ⚠️⚠️ 名字解析必須是**斷言**,不能靠「反正 exec 得起來」。實打實抓到過:
    #    被測碼寫 `GRAV` 而模組層只有 `G0`(GRAV 是 _pn_cmd 的區域變數)
    #    ⇒ 那一行是 NameError,只是被上游的 return 遮住了。
    assert "G0" in ns, "tot_node.py 找不到模組層的 G0"
    assert abs(ns["G0"] - G0) < 1e-9, "G0 變了,本檔的運動學基準要跟著改"
    body = []
    for nm in names:
        body += ["    " + ln for ln
                 in ast.get_source_segment(SRC, _func(nm)).splitlines()]
        body.append("")
    exec("class _M:\n" + "\n".join(body), ns)                  # noqa: S102
    return ns["_M"]


_LT = _mixin("_lat_track_row", "_eta_map")


class Node(_LT):
    """只提供 _lat_track_row 真正碰得到的東西。其餘一律不給 —— 給了就等於
    默許被測碼去碰它,而那正是要防的。"""

    def __init__(self, n=2, gate=6600.0):
        self.n = n
        self.vel = {}
        self.state_t = {}
        self.att = {}
        self.gs = {}
        self.chi_prev = {}
        self.chi_prev_st = {}
        self.weave_rev = {}
        self._rows = {}          # i -> _coord_row 要回傳的 dict
        self._gate = gate

    # 樁:_lat_track_row 與 _eta_map 都只透過這一個介面拿共識狀態。
    def _coord_row(self, i, now):
        return self._rows.get(i)

    def row(self, i, r_to_gate=None, gs=None, r_to_target=None):
        self._rows[i] = {"uav_id": i, "r_to_gate_m": r_to_gate,
                         "gs_mps": gs, "r_to_target_m": r_to_target,
                         "agl_m": 500.0, "released": False,
                         "stamp_age_s": 0.05}
        return self

    def fly(self, i, vn, ve, roll_deg=0.0, st=1.0):
        self.vel[i] = (vn, ve, 0.0)
        self.state_t[i] = st
        self.att[i] = (math.radians(roll_deg), 0.0, 0.0)
        return self


# ═══════════════════════════════════════════════════════════════════════════
# ① 實得側 —— 三個陷阱各一組
# ═══════════════════════════════════════════════════════════════════════════
class TestL1TheHeadingItselfIsMeasuredNotReconstructed:
    """⭐ χ 直接從速度向量取,不從軌跡反推。

    [[release-heading-drives-dive-time]] 那次分析,4 架乾淨機體的航向是從
    軌跡**反推**出來的,因為當時沒有欄位 —— 而反推需要選微分窗,而窗的選擇
    會改變答案。這一欄把它變成量測。
    """

    @pytest.mark.parametrize("vn,ve,want", [
        (100.0, 0.0, 0.0),        # 正北
        (0.0, 100.0, 90.0),       # 正東
        (-100.0, 0.0, 180.0),     # 正南
        (0.0, -100.0, -90.0),     # 正西
        (70.71, 70.71, 45.0),     # 東北
    ])
    def test_chi_is_atan2_east_over_north(self, vn, ve, want):
        r = Node().fly(1, vn, ve)._lat_track_row(1, 1.0)
        assert r["chi_deg"] == pytest.approx(want, abs=0.02), (
            "⚠️ 航跡角的軸向錯了。NED 之下 χ = atan2(vE, vN):0=正北、"
            "+90=正東。把兩個引數對調會讓整批資料鏡射,而鏡射後的分佈"
            "**看起來完全正常**。")

    def test_a_stationary_aircraft_has_no_heading_at_all(self):
        """⚠️ 地面上寫**空字串不是 0** —— 0 是合法航向(正北)。"""
        r = Node().fly(1, 0.3, 0.2)._lat_track_row(1, 1.0)
        assert r["chi_deg"] is None, (
            "⚠️⚠️ 近靜止時 χ 沒有意義,必須是 None(CSV 寫空字串)。"
            "寫 0 會讓事後判讀把一整段停在跑道上的資料讀成「持續朝正北飛」"
            "—— [[rep_interactive]] 那次 UAV1 整趟沒動而閘門 4 秒就通過,"
            "就是同一類誤讀。")


class TestL2TheClockIsTheStateClockNotTheWallClock:
    """⭐⭐⭐ 本檔最重要的一組。

    [[leader-state-freeze]]:長機位姿凍結但仍以 10 Hz 廣播,「停發有告警,
    凍結沒有」。拿牆鐘去微分一份凍結的狀態 ⇒ χ̇ = 0 ⇒ 判讀成「這架沒在
    轉彎」,而真相是遙測沒更新。
    ⇒ 而這個誤讀的方向會讓我們**把好機體判成拉不動**,也就是做出跟事實
      相反的決定。這是本檔存在的主要理由。
    """

    def test_a_frozen_state_yields_no_rate_at_all(self):
        n = Node()
        n.fly(1, 100.0, 0.0, st=10.0)
        n._lat_track_row(1, 100.0)          # 第一拍:建基準
        # 狀態時戳**沒有推進**(位姿凍結),牆鐘照走
        n.vel[1] = (0.0, 100.0, 0.0)        # 就算速度看起來變了
        r = n._lat_track_row(1, 101.0)      # now 前進了一整秒
        assert r["chi_dot_meas_dps"] is None, (
            "⚠️⚠️⚠️ state_t 沒推進時**不可以**產出航向率。\n"
            "    這一拍沒有新資料,任何率都是憑空生出來的。\n"
            "    而它會被讀成「這架轉不動」⇒ 我們會去修一個不存在的問題,\n"
            "    同時錯過真正的問題(遙測凍結)。")
        assert r["chi_dt_s"] is None

    def test_the_dt_is_the_state_delta_not_the_wall_delta(self):
        n = Node()
        n.fly(1, 100.0, 0.0, st=10.0)
        n._lat_track_row(1, 100.0)
        n.fly(1, 0.0, 100.0, st=10.5)       # 狀態推進 0.5 s
        r = n._lat_track_row(1, 108.0)      # 牆鐘推進 8 s
        assert r["chi_dt_s"] == pytest.approx(0.5, abs=1e-6), (
            "⚠️⚠️ dt 必須是 state_t 的差(0.5 s),不是牆鐘的差(8 s)。\n"
            "    用牆鐘會把率**低估 16 倍**,而低估的方向正好是\n"
            "    「看起來拉不動」。")
        # 90° / 0.5 s = 180 °/s
        assert r["chi_dot_meas_dps"] == pytest.approx(180.0, abs=0.1)

    def test_the_window_length_is_recorded_so_artefacts_are_visible(self):
        """⚠️ 3600 行量到實得側的**尾巴是微分窗假象**(>3g 佔比隨半窗 1→16
        從 2.01% 掉到 0.71%),中位數才是窗不變的。⇒ 沒有 chi_dt_s 就分不出
        「真的轉得快」與「窗太短把雜訊放大」。"""
        n = Node()
        n.fly(1, 100.0, 0.0, st=1.0)
        n._lat_track_row(1, 1.0)
        n.fly(1, 10.0, 100.0, st=1.02)
        r = n._lat_track_row(1, 1.02)
        assert r["chi_dt_s"] is not None and r["chi_dt_s"] > 0.0, (
            "⚠️ 每一筆實得率都必須帶著它用的微分窗。少了它,"
            "整批資料就退化成當初那個「重建」——而重建的尾巴是假的。")


class TestL3TheAngleIsUnwrapped:
    """⚠️ +179° → −179° 是轉了 2°,不是 −358°。"""

    def test_crossing_the_antimeridian_is_a_small_turn(self):
        n = Node()
        # 航向 +179°
        n.fly(1, math.cos(math.radians(179.0)) * 100.0,
              math.sin(math.radians(179.0)) * 100.0, st=1.0)
        n._lat_track_row(1, 1.0)
        # 航向 −179°(往同一個方向再轉 2°)
        n.fly(1, math.cos(math.radians(-179.0)) * 100.0,
              math.sin(math.radians(-179.0)) * 100.0, st=2.0)
        r = n._lat_track_row(1, 2.0)
        assert abs(r["chi_dot_meas_dps"]) < 5.0, (
            "⚠️⚠️ 沒有去纏繞:量到 %s °/s。\n"
            "    ±180° 每跨一次就噴一個 ±358 °/s 的假尖峰,而那種尖峰會\n"
            "    污染任何取平均的統計 —— 而我們正要用這批資料算追蹤增益。"
            % r["chi_dot_meas_dps"])


class TestL4BankIsConvertedWithTheMeasuredAngleNotTheCommandedOne:
    """⭐⭐ 這一層測的是「坡度**轉不轉得出**航向」,而它必須用**實測**坡度。

    用指令坡度算 chi_dot_kin,等於把第一層(坡度拉不拉得出來)的缺陷
    偷偷折進第二層 ⇒ 兩層都塌掉時比值仍然是 1,看起來一切正常。
    """

    @pytest.mark.parametrize("roll_deg", [10.0, 30.0, 45.0, -30.0])
    def test_kinematic_rate_is_g_tan_phi_over_vh(self, roll_deg):
        vh = 157.0
        r = Node().fly(1, vh, 0.0, roll_deg=roll_deg)._lat_track_row(1, 1.0)
        want = math.degrees(G0 * math.tan(math.radians(roll_deg)) / vh)
        # ⚠️ 容差 5e-4 = 欄位記錄的精度(小數 3 位),不是「算得不準」。
        #    斷言必須對著**記下來的值**,因為判讀讀的就是那個 ——
        #    對著未捨入的中間值斷言,會在有人調整記錄精度時紅得莫名其妙,
        #    而那正是讓人把鎖停掉的理由。
        assert r["chi_dot_kin_dps"] == pytest.approx(want, abs=5e-4), (
            "⚠️ 協調轉彎的運動學是 χ̇ = g·tanφ / vh。"
            "少了 vh(或用了總速而非水平速)會讓比值隨速度漂,"
            "而漂出來的曲線很像一個「與速度有關的追蹤缺陷」。")

    def test_the_bank_is_clamped_before_the_tangent_diverges(self):
        """⚠️ tan 在 ±90° 發散。不夾的話一筆 89.9° 的坡度就會產出一個
        1e5 °/s 的值,而它會把任何比值統計拉爆。"""
        r = Node().fly(1, 157.0, 0.0, roll_deg=89.9)._lat_track_row(1, 1.0)
        assert r["chi_dot_kin_dps"] is not None
        assert abs(r["chi_dot_kin_dps"]) < 500.0, (
            "⚠️⚠️ 坡度沒有夾在 ±85°:量到 %s °/s。"
            % r["chi_dot_kin_dps"])

    def test_no_attitude_means_no_kinematic_rate_but_chi_survives(self):
        """⚠️ 姿態沒到不代表航向沒到 —— 兩者來自不同的訊息
        (ATTITUDE vs GLOBAL_POSITION_INT)。少一個不可以拖垮另一個。"""
        n = Node()
        n.vel[1] = (100.0, 0.0, 0.0)
        n.state_t[1] = 1.0
        r = n._lat_track_row(1, 1.0)
        assert r["chi_deg"] is not None
        assert r["chi_dot_kin_dps"] is None
        assert r["roll_meas_deg"] is None


# ═══════════════════════════════════════════════════════════════════════════
# ② 蛇形的**窗口**那一半
# ═══════════════════════════════════════════════════════════════════════════
class TestL5TheWindowIsMeasuredNotPredicted:
    """⭐⭐ 實測吸收量 2.85 s 之所以歸不了因,是因為權限側有記
    (itcg_req_a / itcg_cmd_a / itcg_sat / a_h_cmd_g / lat_share)而**窗口側
    一欄都沒有** ⇒ 分不出是「拉不動」還是「沒時間拉」。"""

    def test_the_window_is_range_over_measured_groundspeed(self):
        n = Node().row(1, r_to_gate=3140.0, gs=157.0)
        n.fly(1, 157.0, 0.0)
        r = n._lat_track_row(1, 1.0)
        assert r["weave_window_s"] == pytest.approx(20.0, abs=0.01), (
            "⚠️ 窗口必須是 r_to_gate / **實測地速**。\n"
            "    走預測器的話這一欄會繼承那個中位 +18.7 s 的低估,\n"
            "    而 Zarchan 附錄 A.1 說 tgo 尺度誤差 0.9 就輸給普通 PN。")

    def test_a_stopped_aircraft_has_no_window_rather_than_an_infinite_one(self):
        n = Node().row(1, r_to_gate=3140.0, gs=0.4)
        n.fly(1, 0.4, 0.0)
        r = n._lat_track_row(1, 1.0)
        assert r["weave_window_s"] is None, (
            "⚠️ 地速趨零時 r/gs 會炸成一個巨大的窗口,而那會讓判讀以為"
            "「時間多得是,所以不是窗口問題」——結論剛好反了。")


class TestL6ReversalsAreCountedOnlyWhenItActuallyReverses:
    """⭐ 離線重建說蛇形跑了 **0.21 個週期、零換向** ⇒ 那是斜飛不是蛇形。
    這一欄把那句話從重建變成量測。"""

    def test_the_counter_is_reported_when_present(self):
        n = Node()
        n.fly(1, 100.0, 0.0)
        n.weave_rev[1] = 3
        assert n._lat_track_row(1, 1.0)["weave_rev_n"] == 3

    def test_never_weaved_is_absent_not_zero(self):
        """⚠️ 從沒擺過寫**空字串不是 0**。0 是合法值(擺了但沒換向),
        用 0 當「沒跑」會把兩件事壓成同一個字元 —— 而它們的意義相反:
        「零換向」是 ② 要找的證據,「沒在跑」是資料缺漏。"""
        n = Node()
        n.fly(1, 100.0, 0.0)
        assert n._lat_track_row(1, 1.0)["weave_rev_n"] is None

    # ── 以下三條驅動 `_weave_sign_of` —— 換向真正發生的地方 ────────────
    #
    # ⚠️⚠️⚠️ 這一段是重寫過的,而重寫的理由值得留著:
    #    計數器第一版被我放進 `_weave_theta_of`,而那一支回傳的是
    #    `acos(...)` 的**量值,恆 ≥ 0** —— 沒有正負可言。
    #    ⇒ 計數器**永遠不可能加一**,那一欄會恆空,而恆空會被讀成
    #      「量到零換向」,正好是 ② 想驗證的假說。
    #    **一個不可能產出該結果的儀表,拿去確認假說比不量更糟。**
    #    抓到它的是測試自己的守衛(`assert revs > 0, "這個掃描沒有換向"`),
    #    不是我 —— 所以每一條測試都要有那種「先確認實驗真的跑起來」的斷言。

    @staticmethod
    def _signer():
        w = _mixin("_weave_sign_of")

        class S(w):
            def __init__(self):
                self.t0 = 0.0
                self.weave_period = 20.0
                self._wv_sign = {}
                self.weave_rev = {}

        return S()

    def test_the_clock_really_does_flip_within_one_period(self):
        """⭐ 先證明這組數字真的跨過換向點,否則後面兩條什麼都沒測到。"""
        s = self._signer()
        a = s._weave_sign_of(1, 1.0, 5.0)      # sin(2π·1/20) > 0
        b = s._weave_sign_of(1, 11.0, 5.0)     # sin(2π·11/20) < 0
        assert a > 0 > b, (
            "⚠️ 週期 20 s 之下 t=1 與 t=11 應該落在時鐘的兩側,實得 %s / %s。"
            % (a, b))

    def test_a_real_reversal_while_weaving_is_counted(self):
        s = self._signer()
        s._weave_sign_of(1, 1.0, 5.0)          # 擺,右
        s._weave_sign_of(1, 11.0, 5.0)         # 擺,左  ⇒ 真的換向
        assert s.weave_rev.get(1) == 1, (
            "⚠️⚠️ 真的換向沒有被算到(實得 %s)。這一欄是 ② 的核心證據:"
            "離線重建說『0.21 個週期、零換向』,而只有這個計數器能把那句話"
            "從重建變成量測。數不到就等於沒有證據。"
            % s.weave_rev.get(1))

    def test_a_clock_flip_that_happens_inside_the_deadband_is_not_counted(self):
        """⭐⭐ 語意的關鍵:時鐘一直在翻,但飛機在死區裡**沒有擺**,
        那段時間的翻面沒有發生在軌跡上。

        算進去的後果很具體:一架從未擺動、只在死區進出的機體週期數會
        看起來很高 ⇒ ② 會得到「窗口夠、是拉不動」,而真相剛好相反。
        """
        s = self._signer()
        s._weave_sign_of(1, 1.0, 5.0)          # 擺,右
        s._weave_sign_of(1, 11.0, 0.0)         # 死區:時鐘翻了,但沒擺
        s._weave_sign_of(1, 21.0, 5.0)         # 又擺,而時鐘轉回同一側
        assert s.weave_rev.get(1, 0) == 0, (
            "⚠️⚠️ 死區裡的時鐘翻面被算成了 %d 次換向。"
            "死區是『沒在擺』,不是『擺到另一邊』。"
            % s.weave_rev.get(1, 0))

    def test_crossing_the_deadband_to_the_other_side_IS_counted(self):
        """⚠️ 與上一條配對:死區**中間**的翻面不算,但真的換到另一側要算。
        只有上一條的話,把整個計數器刪掉也會全綠。"""
        s = self._signer()
        s._weave_sign_of(1, 1.0, 5.0)          # 擺,右
        s._weave_sign_of(1, 6.0, 0.0)          # 死區
        s._weave_sign_of(1, 11.0, 5.0)         # 擺,左 ⇒ 軌跡確實換了邊
        assert s.weave_rev.get(1) == 1, (
            "⚠️⚠️ 跨過死區換到另一側**是**一次換向,沒算到(實得 %s)。"
            % s.weave_rev.get(1))

    def test_the_sign_itself_is_not_disturbed_by_the_counting(self):
        """⭐⭐⭐ 只觀測的儀表必須是唯讀的。符號一旦被計數器動到,
        這一趟的資料就不能拿來跟關著的航次比 —— 而那是這趟唯一的用途。"""
        a = self._signer()
        b = self._signer()
        b._wv_sign = None                      # 計數器整個關掉
        for t in (1.0, 6.0, 11.0, 16.0, 21.0):
            assert a._weave_sign_of(1, t, 5.0) == b._weave_sign_of(1, t, 5.0), (
                "⚠️⚠️⚠️ 換向計數改變了送出去的符號(t=%s)。" % t)


# ═══════════════════════════════════════════════════════════════════════════
# ③ ring_pace 之後**還剩下的**殘差
# ═══════════════════════════════════════════════════════════════════════════
class TestL7TheResidualIsRelativeToTheLatestAircraft:
    """⭐⭐ ring_pace 對齊的是「全隊最晚抵環者」,所以殘差必須對 max 取,
    不是對平均或 min。

    ⭐ 文獻對位:橫向偏置是**單向**的 —— 負偏置只能把前置角壓回 0(直線 PN),
       那已經是最短時間,再負也縮不短 ⇒ 共識點必須取**可達下界的 max**。
       取平均會讓較慢的那一架收到不可達的指令,而它會整段釘在夾限
       —— 那正是 [[weave-fails-by-saturation]] 記載的失效。
    """

    def test_the_latest_aircraft_has_exactly_zero_residual(self):
        n = Node(n=3)
        n.row(1, r_to_gate=1570.0, gs=157.0)      # eta 10
        n.row(2, r_to_gate=3140.0, gs=157.0)      # eta 20  <- 最晚
        n.row(3, r_to_gate=2355.0, gs=157.0)      # eta 15
        for i in (1, 2, 3):
            n.fly(i, 157.0, 0.0)
        got = {i: n._lat_track_row(i, 1.0)["pace_eta_resid_s"]
               for i in (1, 2, 3)}
        assert got[2] == pytest.approx(0.0, abs=1e-9), (
            "⚠️⚠️ 最晚的那一架殘差必須恰好 0(它不需要再等)。"
            "不是 0 就表示參考點取錯了。")
        assert got[1] == pytest.approx(10.0, abs=1e-6)
        assert got[3] == pytest.approx(5.0, abs=1e-6)

    def test_the_residual_is_never_negative(self):
        """⚠️ 恆 >= 0。出現負值 = 參考點取成 min 或平均 ⇒ 會有機體收到
        「你要更快」的指令,而那是不可達的。"""
        n = Node(n=2)
        n.row(1, r_to_gate=100.0, gs=157.0)
        n.row(2, r_to_gate=9999.0, gs=157.0)
        for i in (1, 2):
            n.fly(i, 157.0, 0.0)
        for i in (1, 2):
            v = n._lat_track_row(i, 1.0)["pace_eta_resid_s"]
            assert v >= 0.0, "⚠️⚠️ 殘差出現負值 (%s) ⇒ 參考點不是 max" % v

    def test_a_lone_aircraft_has_no_residual_rather_than_zero(self):
        """⚠️ 只有一架時**沒有**散布可言,寫空字串不是 0 —— 0 會被讀成
        「已經對齊」,而那是 n=1 的假象。"""
        n = Node(n=1)
        n.row(1, r_to_gate=3140.0, gs=157.0)
        n.fly(1, 157.0, 0.0)
        assert n._lat_track_row(1, 1.0)["pace_eta_resid_s"] is None


class TestL8TheReleaseGeometryPairIsFinallyComplete:
    """⭐⭐ [[release-heading-drives-dive-time]]:俯衝耗時由**航向與距離兩個**
    決定(偏相關 +0.873 / +0.868),兩個都要鎖。

    σ 一直有欄位(lead_angle_deg),**R 沒有** ⇒ 這一對從來配不起來。
    """

    def test_range_to_target_is_carried_through(self):
        n = Node().row(1, r_to_gate=3140.0, gs=157.0, r_to_target=6760.0)
        n.fly(1, 157.0, 0.0)
        assert n._lat_track_row(1, 1.0)["r_to_target_m"] == 6760.0

    def test_no_target_means_no_range_rather_than_zero(self):
        """⚠️ 待命(目標是 NaN)時寫空字串不是 0 —— 0 公尺是「已經到了」。"""
        n = Node().row(1, r_to_gate=None, gs=157.0, r_to_target=None)
        n.fly(1, 157.0, 0.0)
        assert n._lat_track_row(1, 1.0)["r_to_target_m"] is None


# ═══════════════════════════════════════════════════════════════════════════
# 跨切面:儀表不可以改變被量測的東西
# ═══════════════════════════════════════════════════════════════════════════
class TestL9TheInstrumentDoesNotPerturbWhatItMeasures:
    """⭐⭐⭐ 只觀測的儀表**必須**是唯讀的。

    ⚠️ 我加換向計數時第一版直接寫 `self._wv_sign`,而本檔到處都是跳過
       __init__ 的樁節點 ⇒ 27 條測試立刻 AttributeError。
       **儀表讓被量測的那條路失敗,是最糟的一種失敗**:它同時毀掉航次
       與資料,而且看起來像被測物的錯。
    """

    def test_the_weave_angle_is_unchanged_by_the_counter(self):
        """換向計數只能讀 th,不能改它。"""
        w = _mixin("_weave_theta_of")

        class W(w):
            def __init__(self):
                self.weave_time_sync = True
                self.weave_relative = False
                self.weave_theta = {}
                self.weave_rev = {}
                self._wv_sign = {}
                self.e_t = {}
                self.e_t_rel = {}
                self.weave_theta_max = 45.0
                self.weave_deadband_s = 0.5
                self.weave_period = 22.0
                self.dive_T = 100.0
                self.pos = {1: (0.0, 0.0, 3100.0)}
                self.gs = {1: 157.0}
                self.T = (0.0, 0.5)
                self.ring_gate_frac = 1.1

            def ring_of(self, i):
                return 6000.0

            def _tgo_of(self, i, d):
                return 40.0

        a = W()
        first = a._weave_theta_of(1, 50.0, 8000.0)
        b = W()
        b._wv_sign = None                    # 計數器整個關掉
        second = b._weave_theta_of(1, 50.0, 8000.0)
        assert first == second, (
            "⚠️⚠️⚠️ 換向計數改變了送出去的偏航角(%s vs %s)。\n"
            "    只觀測的儀表一旦動到指令,這一整趟的資料就不能拿來\n"
            "    跟關著的航次比 —— 而那正是這趟航次唯一的用途。"
            % (first, second))

    def test_a_stub_node_without_init_does_not_explode(self):
        """⚠️ 樁節點跳過 __init__ ⇒ 容器根本不存在。儀表必須自己擋住,
        而不是讓呼叫端到處包 try(包了會把真正的 bug 一起吞掉)。"""
        w = _mixin("_weave_theta_of")

        class Bare(w):
            weave_time_sync = False
            weave_theta = {}

        b = Bare()
        b._weave_theta_of(1, 0.0, 100.0)     # 不可以拋例外
