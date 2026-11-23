# -*- coding: utf-8 -*-
"""t_go 時基污染的迴歸鎖(RPY3,2026-08-14)。

═══ 這一批鎖在防什麼 ═══

`_tgo_of()` 舊簽章是 `(self, i, d, now=None)`。九個呼叫點裡:
  * 兩個傳 `time.time()`(**牆鐘**,約 1.7867e9)
  * 七個讓它預設成 `time.monotonic()`(**開機時鐘**,約 1e3)
而兩者**共用同一個 `tgo_cache`**。於是:

  ① `_observe_timing` 用牆鐘寫入 cache = (1.7867e9, t_go)
  ② 抵環重排用開機時鐘讀:`monotonic − epoch` = **−17.8 億**
  ③ 新鮮度判斷 `−17.8億 < 0.5` ⇒ 誤判「快取還新鮮」
  ④ 外推 `cached[1] − (−17.8億)` = **17.8 億秒**
  ⑤ `dive_T = now + 17.8億 + 2` ⇒ 共同命中時刻排到 **56 年後**

⚠️⚠️ 兩個錯誤剛好**互相掩護**:負的經過時間同時讓新鮮度通過、又讓外推變成
   加法。任何一邊單獨都不會出事,合起來就吐出一個天文數字而且完全無聲。

後果全部靜默,而且是三條路一起死:
  * `t_go >= dive_T − now` 的時刻表放行 ⇒ 永不成立 ⇒ 放行退化成只剩幾何守門員
    (誰先撞到俯角上限誰先走 = **任意順序**)
  * `v_req = R/(dive_T − now)` ≈ 0 ⇒ 夾到 vmin ⇒ 俯衝段**零時間權限**
  * `_miss_check` 的 `now > dive_T + window` ⇒ 永不成立 ⇒ miss 出口死掉

實測代價:RPY3 散布 **78.5 s**,同組態的 RPY1 卻只有 12.1 s —— 重現性全失,
而且看起來像「導引律不穩」,把歸因整個帶錯方向。

⚠️ log 當時**就把數字印出來了**:
   「★ 首架抵環 → 重排共同命中時刻:T+1786674677.0s(舊表已過期 0s)」
   格式正常、沒有告警、沒有人讀。所以攔截必須放在**寫入點**,不能靠人眼。
"""
import ast
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

EPOCH = 1786674539.8          # RPY3 當下的牆鐘值(就是洩漏出來的那個數)
MONO = 4213.7                 # 同一時刻的開機時鐘,量級差 6 個數量級


def _func(name):
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"tot_node.py 找不到 {name}()")


class _Stub:
    """只帶 `_tgo_of` 需要的最小狀態。

    ⚠️ 刻意**不** import rclpy:這一層的邏輯與 ROS2 無關,而把它綁死在
       ROS2 上會讓這個鎖只能在 WSL 跑 —— 那等於沒有鎖。
    """

    tgo_from_predictor = True
    tgo_predict_dt = 0.5
    # ⚠️ 這一組鎖測的是**時基**,不是進環時間 —— 把 ring transit 關掉,
    #    否則兩個機制混在同一條斷言裡,壞掉時分不出是哪個。
    tgo_ring_transit = False
    dive_ring_gate = True
    ring_gate_frac = 1.10
    RING_CLOSE_WINDOW_S = 20.0
    RING_CLOSE_MIN_MS = 10.0
    predict_gamma_cap = 0.0
    predict_thr_max = 25.0
    T = (25.1261649, 120.615648)
    _tgo_warned = False

    def __init__(self):
        self.tgo_cache = {}
        self.d_hist = {}
        self.zg_amsl = {1: 0.0}
        self.pace = 150.0

    def _pace_of(self, i, d):
        return self.pace

    def ring_of(self, i):
        return 6084.0

    def _geom(self, i):
        # dn, de, d, h, R, vn, ve, vh, hdot, V
        return (0.0, 4000.0, 4000.0, 3000.0, 5000.0, 0.0, 150.0, 150.0, -20.0, 152.0)

    def _log_err(self, *a, **k):
        pass

    def get_logger(self):
        return type("L", (), {"error": lambda *a, **k: None,
                              "warn": lambda *a, **k: None,
                              "info": lambda *a, **k: None})()


class _FakePred:
    """⚠️⚠️ 這個假預測器**必須存在**。

    第一版把 `_pred` 設成 None,於是 `_tgo_of` 在第一行
    `if not self.tgo_from_predictor or _pred is None ...: return fallback`
    就退出了 —— 測試全綠,卻**一次都沒走到快取那條路**。
    測試通過得太早比沒有測試更糟:它會讓人以為那條路被鎖住了。
    """

    PN_GAMMA_CAP_DEG = 36.0
    FRESH_T = 30.0

    @staticmethod
    def predict_terminal(h, d, V, **kw):
        return {"t_at_crossing": _FakePred.FRESH_T, "h_at_crossing": 0.0,
                "gs_end": 190.0, "t_go": _FakePred.FRESH_T,
                "gamma_sat_frac": 0.0}


def _bind():
    """把 tot_node 的 `_tgo_of` / `_set_dive_T` 綁到樁上,不啟動 ROS2。"""
    ns = {}
    src = ast.get_source_segment(SRC, _func("_tgo_of"))
    dts = ast.get_source_segment(SRC, _func("_set_dive_T"))
    rts = ast.get_source_segment(SRC, _func("_ring_transit_s"))
    import time as _time
    ns.update({"time": _time, "math": math, "_pred": _FakePred})
    exec("class _M:\n" + "\n".join("    " + l for l in src.splitlines())
         + "\n" + "\n".join("    " + l for l in dts.splitlines())
         + "\n" + "\n".join("    " + l for l in rts.splitlines())
         + "\n    DIVE_T_HORIZON_MAX_S = 3600.0\n", ns)
    return ns["_M"]


class TestCallersCannotInjectAClock:
    """⭐⭐ 最硬的一條:時基做成**結構上不可注入**,不靠呼叫端自律。

    ⚠️ 「以後記得都傳同一種時鐘」是行不通的 —— RPY3 之前也沒人打算混用,
       九個呼叫點是逐步長出來的,每一個單看都合理。
    """

    def test_tgo_of_signature_has_no_clock_parameter(self):
        args = [a.arg for a in _func("_tgo_of").args.args]
        assert args == ["self", "i", "d"], (
            f"⚠️⚠️ `_tgo_of` 的簽章變成 {args}。\n"
            f"    只要它收得到時鐘,就有兩個呼叫端傳不同時鐘的可能,\n"
            f"    而 tgo_cache 是共用的 ⇒ RPY3 的 56 年時刻表會原樣回來。\n"
            f"    節流時鐘必須只在函式內部取一次 time.monotonic()。")

    def test_no_call_site_passes_a_third_argument(self):
        bad = []
        for node in ast.walk(TREE):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, "attr", None) == "_tgo_of"
                    and (len(node.args) > 2 or node.keywords)):
                bad.append(node.lineno)
        assert not bad, f"⚠️ 這些行仍在對 _tgo_of 注入時鐘:{bad}"

    def test_the_cache_is_written_with_the_internal_clock_only(self):
        """快取的時戳只能來自函式內部那一次 monotonic。"""
        fn = _func("_tgo_of")
        writes = [n for n in ast.walk(fn)
                  if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Subscript)
                          and getattr(t.value, "attr", None) == "tgo_cache"
                          for t in n.targets)]
        assert writes, "找不到 tgo_cache 的寫入點"
        for w in writes:
            names = {n.id for n in ast.walk(w.value) if isinstance(n, ast.Name)}
            assert "mono" in names, (
                "⚠️ tgo_cache 的時戳不是用內部的 `mono` 寫入 —— "
                "只要它來自參數,兩種時鐘就會再次混進同一個快取")


class TestAPollutedCacheCannotInflateTgo:
    """⭐ 行為鎖:就算快取真的被污染,也不准吐出荒謬值。

    AST 鎖擋的是「簽章退化」,這一條擋的是「換了另一種方式再污染一次」。
    """

    def test_negative_age_is_treated_as_pollution_not_freshness(self):
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.tgo_cache[1] = (EPOCH, 30.0)      # 牆鐘寫入的快取
        got = s._tgo_of(1, 4000.0)          # 內部用 monotonic 讀
        assert 0.0 < got < 600.0, (
            f"⚠️⚠️ 污染的快取吐出 t_go = {got:.1f}s。\n"
            f"    RPY3 就是這樣得到 17.8 億秒的:負的經過時間同時讓\n"
            f"    新鮮度判斷通過、又讓外推變成加法。")
        assert got == pytest.approx(_FakePred.FRESH_T), (
            "污染時應丟掉快取重算,而不是拿污染值外推")

    def test_the_cache_timestamp_is_never_epoch_magnitude(self):
        """⭐⭐ 這是整組鎖的核心不變式,而且與實作細節無關。

        `tgo_cache` 的時戳只准是開機時鐘。任何 1e9 量級的值都代表牆鐘漏進來了,
        而那正是 56 年時刻表的唯一入口。
        """
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.tgo_cache[1] = (EPOCH, 30.0)
        s._tgo_of(1, 4000.0)
        assert 1 in s.tgo_cache, "重算之後應該重新填快取"
        assert s.tgo_cache[1][0] < 1e9, (
            f"⚠️⚠️ 快取時戳 {s.tgo_cache[1][0]} 是牆鐘量級。"
            f"只要它是牆鐘,下一個用 monotonic 的呼叫端就會算出負的經過時間。")

    def test_a_sane_cache_still_extrapolates(self):
        """反向鎖:護欄不可以順手把正常的節流也擋掉。"""
        import time
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.tgo_cache[1] = (time.monotonic() - 0.1, 30.0)
        got = s._tgo_of(1, 4000.0)
        assert got == pytest.approx(29.9, abs=0.05), (
            f"正常快取應線性外推到約 29.9s,實得 {got}")


class TestBothCachePathsAreGuarded:
    """⭐⭐ 護欄必須**兩條路徑都有**。

    ⚠️ RPY3 的根本形狀:新鮮路徑有 `0.0 < t < 600.0`,外推路徑沒有。
       護欄只裝在一條路上,結果就從沒裝的那條漏出去 —— 而且因為另一條
       有裝,程式碼讀起來像是「有在檢查」。
    """

    def test_negative_age_is_explicitly_rejected(self):
        """⚠️ 這一條只能用 AST 鎖,因為它在行為上**被 600s 上界遮蔽**。

        變異測試(2026-08-14)證實:把 `0.0 <= age` 拆掉,12 條測試仍全綠 ——
        負的 age 讓外推值變成 1.78e9,而 `0 < t < 600` 已經先把它攔下來了。
        兩道護欄任一道都足夠,這是刻意的縱深防禦。
        但正因為它在行為上測不出來,拆掉它**不會有任何測試變紅** ⇒
        必須用結構鎖住,否則它會在某次「簡化」中無聲消失,
        而縱深防禦一旦剩一層,就跟 RPY3 之前一樣脆。
        """
        fn = _func("_tgo_of")
        found = False
        for node in ast.walk(fn):
            if not isinstance(node, ast.Compare):
                continue
            names = {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
            zeros = [c.value for c in ast.walk(node)
                     if isinstance(c, ast.Constant) and c.value == 0]
            if "age" in names and zeros and any(
                    isinstance(op, (ast.LtE, ast.Lt)) for op in node.ops):
                found = True
        assert found, (
            "⚠️⚠️ `_tgo_of` 少了「經過時間不得為負」的判斷。\n"
            "    負的 age **本身就是時基被污染的證據**,而且它會同時\n"
            "    (a) 讓新鮮度判斷通過、(b) 讓外推變成加法 —— 兩個錯誤互相掩護。")

    def test_the_bound_appears_at_least_twice(self):
        fn = _func("_tgo_of")
        n = 0
        for node in ast.walk(fn):
            if not isinstance(node, ast.Compare):
                continue
            consts = [c.value for c in ast.walk(node)
                      if isinstance(c, ast.Constant)
                      and isinstance(c.value, (int, float))]
            if 600.0 in consts or 600 in consts:
                n += 1
        assert n >= 2, (
            f"⚠️⚠️ `0 < t < 600` 這道合理性界只出現在 {n} 條路徑上。\n"
            f"    新鮮路徑與外推路徑**都**要有 —— 只裝一邊正是 RPY3 的形狀。")


class TestDiveTHasAnAbsurdityGuard:
    """⭐ 共同命中時刻是放行、速度、miss 出口三條路的共同上游。

    它壞掉的時候三條路一起靜默失效 ⇒ 攔截放在**寫入點**。
    """

    def test_absurd_horizon_is_rejected(self):
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.dive_T = None
        ok = s._set_dive_T(EPOCH, 1786674677.0, "測試")
        assert ok is False, "⚠️⚠️ 56 年後的時刻表被接受了"
        assert s.dive_T is None, "被拒的值不可以寫進去"

    def test_a_normal_horizon_is_accepted(self):
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.dive_T = None
        assert s._set_dive_T(EPOCH, 35.1, "測試") is True
        assert s.dive_T == pytest.approx(EPOCH + 35.1)

    def test_zero_and_negative_are_rejected(self):
        """已經過期的時刻表也是壞的 —— 寫進去等於全隊立刻放行。"""
        M = _bind()
        s = _Stub()
        s.__class__ = type("S", (_Stub, M), {})
        s.dive_T = None
        assert s._set_dive_T(EPOCH, 0.0, "測試") is False
        assert s._set_dive_T(EPOCH, -5.0, "測試") is False

    def test_dive_T_is_never_assigned_outside_the_guard(self):
        """⚠️ 繞過攔截器直接指派 ⇒ 攔截等於沒有。"""
        bad = []
        for node in ast.walk(TREE):
            if not isinstance(node, ast.Assign):
                continue
            for t in node.targets:
                if (isinstance(t, ast.Attribute) and t.attr == "dive_T"
                        and getattr(t.value, "id", None) == "self"):
                    bad.append(node.lineno)
        inside = {n.lineno for n in ast.walk(_func("_set_dive_T"))
                  if hasattr(n, "lineno")}
        init = {n.lineno for n in ast.walk(TREE)
                if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Attribute) and t.attr == "dive_T"
                        for t in n.targets)
                and isinstance(n.value, ast.Constant) and n.value.value is None}
        stray = [l for l in bad if l not in inside and l not in init]
        assert not stray, (
            f"⚠️⚠️ 這些行直接指派 self.dive_T,繞過了荒謬值攔截:{stray}\n"
            f"    一律改走 `self._set_dive_T(now, horizon_s, why)`。")


class TestStalenessIsMeasuredBeforeOverwrite:
    """⚠️ 「舊表已過期 0s」—— 那行本來就該暴露問題,卻算在覆寫之後。

    先覆寫再算差值,差值恆為 0,於是最該刺眼的一行長得一切正常。
    """

    def test_stale_is_computed_from_the_old_dive_T(self):
        src = SRC
        i_stale = src.find("stale = (None if self.dive_T is None")
        i_set = src.find('self._set_dive_T(now, t_need + 2.0, "首架抵環重排")')
        assert i_stale != -1 and i_set != -1, "抵環重排的過期量計算不見了"
        assert i_stale < i_set, (
            "⚠️⚠️ 過期量必須在 _set_dive_T 覆寫**之前**算。"
            "算在之後 ⇒ 恆為 0 ⇒ RPY3 印出的『舊表已過期 0s』會再來一次。")
