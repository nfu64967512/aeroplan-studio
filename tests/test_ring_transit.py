# -*- coding: utf-8 -*-
"""進環時間併入 t_go 的迴歸鎖(RPZ1,2026-08-15)。

═══ 為什麼需要這一層 ═══

放行有**兩個條件串在一起**:
  ① 空間閘門 —— 必須先飛進 pushover 環(半徑 × ring_gate_frac)
  ② 時間時刻表 —— `t_go_i >= dive_T − now`

而 `dive_T = max_j(t_go_j) + 2`,裡面的 `t_go` 算的是「直飛紅心要多久」,
**完全沒有把①算進去**。於是還在環外的機體即使時刻表早已輪到它也不能放行。

實測(RPZ1):
  * UAV1 在環內 4964 m ⇒ 立刻放行
  * UAV2 在環外 7543 m(超出閘門 6692 m 共 851 m)⇒ 等了 **37 s**
  * 放行時刻差 **+37.2 s**,而同一趟的**俯衝耗時差只有 −1.0 s**
  ⇒ 下游(俯衝段)在前一輪已經修好,散布整個搬到上游的放行段。

對照 RPZ2:兩架都在環內(5360 / 5615 m)⇒ 放行差 −2.5 s ⇒ 散布 **3.9 s**。

文獻依據(Bin et al.,*Impact time control guidance against maneuvering
targets based on a nonlinear virtual relative model*,式 49):
    T_d > r_go|_{C_M=0, r=r_0} / V_R
可行命中時刻的下界由「**當下狀態可飛的最短幾何航程**」決定,不是直線距離。
有空間閘門時,那個航程 = 到環的航程 + 環內俯衝航程。

⭐ 這條修法之所以只需要動一個函式,是因為 T1 已經把 t_go 收成單一入口:
   `dive_T` 的 max 與逐機的放行比較式走的是同一個 `_tgo_of`,
   不可能出現「兩邊對 t_go 的認知不同」。
"""
import ast
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

RING = 6084.0          # contract derived.ring_radius_m
FRAC = 1.10
GATE = RING * FRAC     # 6692.4 m


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


class _FakePred:
    PN_GAMMA_CAP_DEG = 36.0

    @staticmethod
    def predict_terminal(h, d, V, **kw):
        # 讓「環到紅心」的時間可由距離反推,方便斷言分段是否正確
        return {"t_at_crossing": d / 200.0, "h_at_crossing": 0.0,
                "gs_end": 190.0, "t_go": d / 200.0, "gamma_sat_frac": 0.0}


class _Stub:
    tgo_from_predictor = True
    tgo_predict_dt = 0.5
    tgo_ring_transit = True
    dive_ring_gate = True
    ring_gate_frac = FRAC
    predict_gamma_cap = 0.0
    predict_thr_max = 25.0
    T = (25.1261649, 120.615648)
    _tgo_warned = False

    def __init__(self):
        self.tgo_cache = {}
        self.d_hist = {}
        self.zg_amsl = {1: 0.0}
        self.pace = 150.0
        self._d = 4000.0

    def _pace_of(self, i, d):
        return self.pace

    def ring_of(self, i):
        return RING

    def _geom(self, i):
        return (0.0, self._d, self._d, 3000.0, 5000.0, 0.0, 150.0, 150.0,
                -20.0, 152.0)

    def get_logger(self):
        return type("L", (), {"error": lambda *a, **k: None,
                              "warn": lambda *a, **k: None,
                              "info": lambda *a, **k: None})()


def _bind():
    ns = {}
    import time as _time
    ns.update({"time": _time, "math": math, "_pred": _FakePred})
    body = "\n".join(
        "\n".join("    " + l for l in ast.get_source_segment(SRC, _func(f)).splitlines())
        for f in ("_tgo_of", "_ring_transit_s"))
    exec("class _M:\n" + body
         + "\n    RING_CLOSE_WINDOW_S = 20.0"
         + "\n    RING_CLOSE_MIN_MS = 10.0"
         + "\n    RING_TRANSIT_MAX_S = 300.0"
         + "\n    _ring_transit_warned = False\n", ns)
    return ns["_M"]


def _mk(d=4000.0, **kw):
    M = _bind()
    s = _Stub()
    s.__class__ = type("S", (_Stub, M), {})
    s._d = d
    for k, v in kw.items():
        setattr(s, k, v)
    return s


class TestInsideTheRingIsUnchanged:
    """⭐ 反向鎖:環內的行為必須**逐位元不變**。

    ⚠️ 新機制若順手改了環內的 t_go,就同時動了兩個變數,
       下一趟的對照就不再是單變數 —— 而 RPZ 這批的價值全在同組態對照。
    """

    def test_no_transit_added_inside_the_gate(self):
        s = _mk(d=5000.0)
        assert s._ring_transit_s(1, 5000.0) == 0.0
        assert s._tgo_of(1, 5000.0) == pytest.approx(5000.0 / 200.0)

    def test_exactly_at_the_gate_is_inside(self):
        s = _mk(d=GATE)
        assert s._ring_transit_s(1, GATE) == 0.0

    def test_switch_off_restores_old_behaviour(self):
        """開關關掉 ⇒ 與修改前逐位元相同。"""
        s = _mk(d=9000.0, tgo_ring_transit=False)
        assert s._ring_transit_s(1, 9000.0) == 0.0
        assert s._tgo_of(1, 9000.0) == pytest.approx(9000.0 / 200.0)


class TestOutsideTheRingAddsTransit:
    def test_transit_is_positive_outside(self):
        s = _mk(d=7543.0)
        s._ring_transit_s(1, 7600.0)          # 建立歷史
        assert s._ring_transit_s(1, 7543.0) > 0.0

    def test_the_geometric_leg_is_capped_at_the_gate(self):
        """⚠️⚠️ 幾何段只能算到**環**,不能算到當下位置。

        算整段 d 再另外加 transit ⇒ 「環到現在位置」那一段被算**兩次**,
        t_go 系統性偏大 ⇒ dive_T 太晚 ⇒ 全隊白等。
        """
        s = _mk(d=9000.0)
        s._ring_transit_s(1, 9000.0)
        got = s._tgo_of(1, 9000.0)
        transit = s._ring_transit_s(1, 9000.0)
        # 幾何段必須是 GATE/200,不是 9000/200
        assert got - transit == pytest.approx(GATE / 200.0, rel=0.02), (
            f"幾何段算成 {(got - transit) * 200.0:.0f} m,應為閘門半徑 {GATE:.0f} m —— "
            f"重複計算了環到當下位置那一段")

    def test_fallback_path_also_caps_the_geometric_leg(self):
        """非預測器路徑(靜態幾何配速)必須用同一套分段,否則兩條路不一致。"""
        s = _mk(d=9000.0, tgo_from_predictor=False)
        s._ring_transit_s(1, 9000.0)
        transit = s._ring_transit_s(1, 9000.0)
        got = s._tgo_of(1, 9000.0)
        assert got - transit == pytest.approx(GATE / 150.0, rel=0.02)


class TestClosingRateMustBeWindowedNotInstantaneous:
    """⭐⭐ 這是整條修法成敗的關鍵。

    ⚠️⚠️ RPZ1-UAV2 放行前的**瞬時**徑向速率是 −91 m/s(看起來 9 秒就到),
       但它在蛇形,實際 37 s 只推進 851 m ⇒ **23 m/s**。
       用瞬時值會把進環時間低估 4 倍 —— 那等於這條修法整個白做,
       而且結果會長得像「改了但沒效」,最難查。
    """

    def test_uses_the_whole_window_not_the_last_step(self):
        import time
        s = _mk(d=7543.0)
        now = time.monotonic()
        # 模擬蛇形:整體 20 s 只推進 460 m(23 m/s),但最後一步很快
        s.d_hist[1] = [(now - 20.0, 8003.0), (now - 1.0, 7600.0)]
        t = s._ring_transit_s(1, 7543.0)
        # 窗口速率 ≈ (8003 − 7543)/20 = 23 m/s ⇒ 851/23 ≈ 37 s
        assert t == pytest.approx((7543.0 - GATE) / 23.0, rel=0.15), (
            f"進環時間 {t:.1f}s。若用最後一步(57 m/1 s = 57 m/s)會得到 "
            f"{(7543.0 - GATE) / 57.0:.1f}s —— 低估 2.4 倍。")

    def test_a_slow_or_receding_aircraft_gets_a_bounded_estimate(self):
        """⚠️ 正在**遠離**目標時閉合速率是負的。不夾住會得到負的進環時間,
        而負的 t_go 會讓放行條件立刻成立 —— 剛好與意圖相反。"""
        import time
        s = _mk(d=7543.0)
        now = time.monotonic()
        s.d_hist[1] = [(now - 20.0, 7000.0)]   # 距離在**變大**
        t = s._ring_transit_s(1, 7543.0)
        assert t > 0.0, "遠離目標時進環時間不可以是負的或零"
        assert t == pytest.approx((7543.0 - GATE) / 10.0, rel=0.01), (
            "量不到有效閉合速率時應退到下限 RING_CLOSE_MIN_MS,而且要保守(慢)")

    def test_the_floor_is_conservative_not_optimistic(self):
        """⭐ 估晚 vs 估早**不對稱**:估晚只是多等一會兒(代價是終端速度),
        估早會讓 dive_T 太早、受限那架照樣遲到 ⇒ 散布原樣保留。"""
        s = _mk()
        assert s.RING_CLOSE_MIN_MS <= 20.0, (
            f"閉合速率下限 {s.RING_CLOSE_MIN_MS} m/s 太樂觀。"
            f"RPZ1 實測的有效閉合速率只有 23 m/s,下限必須明顯低於它。")


class TestBoundsStillHold:
    def test_transit_cannot_push_tgo_past_the_sanity_bound(self):
        """⚠️ 加了 transit 之後必須**重驗**上界。只在加之前驗 ⇒ 護欄被繞過。"""
        import time
        s = _mk(d=100000.0)
        now = time.monotonic()
        s.d_hist[1] = [(now - 20.0, 100001.0)]   # 幾乎不動 ⇒ transit 巨大
        got = s._tgo_of(1, 100000.0)
        # ⚠️ 界線必須是**真正的不變式 600 s**,不是隨手挑一個大數。
        #    第一版寫 `< 1e6`,而拿掉重驗之後實得 9364 s —— 遠超 600 卻小於 1e6,
        #    變異測試因此沒被抓到。**寬鬆的斷言等於沒有斷言。**
        assert got < 600.0, (
            f"t_go = {got:.0f}s > 600 —— 加了 transit 之後沒有重驗上界。"
            f"上界只在加之前驗 ⇒ 護欄被繞過,而繞過的量剛好是 transit。")

    def test_sum_over_the_bound_falls_back_even_when_each_part_is_legal(self):
        """⭐⭐ 這條打的是**只有相加才超界**的情形。

        ⚠️ 進環時間有 300 s 的上限、預測值有 600 s 的上限,**各自都合法**,
           但 300 + 500 = 800 已經超界。若只在加 transit **之前**驗,
           這個組合會整個溜過去 —— 而它正是最難察覺的一種:
           兩個部分分開看都「在範圍內」。
        """
        import time
        s = _mk(d=9000.0)
        _FakePred_slow = type("P", (), {
            "PN_GAMMA_CAP_DEG": 36.0,
            "predict_terminal": staticmethod(
                lambda h, d, V, **kw: {"t_at_crossing": 500.0,
                                       "h_at_crossing": 0.0, "gs_end": 190.0,
                                       "t_go": 500.0, "gamma_sat_frac": 0.0})})
        import sys as _sys
        mod = _sys.modules[type(s).__mro__[2].__module__]
        old = getattr(mod, "_pred", None)
        try:
            # 直接換掉 exec 命名空間裡的 _pred
            type(s).__mro__[2]._tgo_of.__globals__["_pred"] = _FakePred_slow
            now = time.monotonic()
            s.d_hist[1] = [(now - 20.0, 9001.0)]     # 幾乎不動 ⇒ transit 夾到 300
            got = s._tgo_of(1, 9000.0)
            assert got < 600.0, (
                f"t_go = {got:.0f}s。進環 300 s(合法)+ 預測 500 s(合法)"
                f"= 800 s(不合法)。上界必須在**相加之後**再驗一次。")
        finally:
            if old is not None:
                type(s).__mro__[2]._tgo_of.__globals__["_pred"] = old

    def test_the_recheck_exists_in_source(self):
        """行為鎖可能被 fallback 遮蔽,所以再加一條結構鎖。"""
        fn = ast.get_source_segment(SRC, _func("_tgo_of"))
        assert fn.count("600.0") >= 3, (
            "⚠️ `0 < t < 600` 的驗證少於三處(新鮮路徑、外推路徑、加 transit 之後)")
