# -*- coding: utf-8 -*-
"""⚗️ 航向閘門(因果實驗專用)的鎖。

═══ 這條路徑為什麼存在 ═══

俯衝耗時的變異 σ=6.46 s 完全解釋了目前的散布(σ√2=9.14 vs 實測 9.48)。
查到的候選成因是**放行瞬間的航向誤差**:27 架次中位 151°(機頭幾乎背對紅心),
迂迴倍率最高 2.74(直線 4173 m 卻飛了 11420 m)。

⚠️ 但乾淨的那 4 架**同時**滿足「航向 27°」與「距離 6760 m」,**完全共線**,
   用現有資料分不開是哪一個。

⭐ 幾何給了一個免費的對照:盤旋圓對 D→T 連線是對稱的,所以
   「σ≈0 的相位集合」與「σ≈180 的相位集合」對應到**同一組距離值** ——
   兩群的距離**分佈中心逐位元重合**(任何容差下都是 0 m 差),
   中心即切線距離 sqrt(D² − a²) = 6008 m(D=6147、a=1300)。
   ⇒ 放兩架分別在兩群,**距離分佈自動匹配,只有航向不同**。

⚠️ 我第一版把這寫成「兩點距離完全相同」,被本檔的測試打臉:
   ±20° 容差下群內距離其實跨 887 m。正確的機制是**分佈匹配**不是**定值**。
   結論仍成立,但強度弱一級 —— 這個區別要留著,不要又講回去。

⚠️ LM 建議的 2×2 全因子(R∈{5000,7000} × σ∈{15°,150°})在這個幾何上取不到:
   σ 與 R 耦合,任一 σ 對應的距離都集中在 6008 m 附近。
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
    raise AssertionError(f"找不到 {name}()")


def _parse():
    ns = {}
    exec("class _M:\n" + "\n".join(
        "    " + l for l in ast.get_source_segment(SRC, _func("_parse_sigma_spec")).splitlines()), ns)
    return ns["_M"]._parse_sigma_spec


class TestSpecParsing:
    def test_empty_means_disabled(self):
        assert _parse()("") == {}
        assert _parse()("   ") == {}

    def test_the_experiment_spec(self):
        assert _parse()("1:0:20,2:180:20") == {1: (0.0, 20.0), 2: (180.0, 20.0)}

    @pytest.mark.parametrize("bad", ["1:0", "1:0:20:9", "x:0:20", "1:0:abc", "1"])
    def test_malformed_drops_the_whole_spec(self, bad):
        """⚠️⚠️ 半套規格比沒有更糟:一半機體走實驗路徑、一半走生產路徑,
        那是**分不出來的髒對照**,而且事後看不出來。"""
        assert _parse()(bad) == {}, f"殘缺規格 {bad!r} 被部分接受了"

    @pytest.mark.parametrize("spec", [
        "1:0:20,BAD",              # 欄位數不對
        "1:0:20,x:180:20",         # ⚠️ 數值解析失敗(sysid 不是整數)
        "1:0:20,2:abc:20",         # ⚠️ 數值解析失敗(角度不是數字)
    ])
    def test_partial_failure_drops_valid_entries_too(self, spec):
        """⚠️⚠️ 必須是**多筆**規格才測得到:單筆壞掉時 `continue` 與 `return {}`
        的結果都是 `{}`,分不出來。變異測試(2026-08-15)證實了這一點 ——
        我第一版只放單筆壞規格,把 `except ValueError: continue` 這個變異放過去了。
        而那個變異的後果正是「一半機體走實驗路徑、一半走生產路徑」。"""
        assert _parse()(spec) == {}, f"{spec!r} 只有一半被接受了"


class TestTheGateIsOnEveryReleasePath:
    """⚠️⚠️ 放行有**兩條**路徑(時刻表、幾何守門員)。閘門只掛一條 ⇒
    實驗會從沒掛的那條漏掉,而漏掉的樣本看起來完全正常。
    這正是 RPY3 那次「護欄只裝在一條路上」的形狀。"""

    def test_both_release_sites_call_the_gate(self):
        hits = [n.lineno for n in ast.walk(TREE)
                if isinstance(n, ast.Call)
                and getattr(n.func, "attr", None) == "_sigma_gate_ok"]
        assert len(hits) >= 2, (
            f"`_sigma_gate_ok` 只在 {len(hits)} 處被呼叫,但放行有兩條路徑")

    def test_no_release_without_the_gate(self):
        """每個 `self.released[i] = True` 都必須在某個提到閘門的區塊裡。"""
        assigns = [n.lineno for n in ast.walk(TREE)
                   if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Subscript)
                           and getattr(t.value, "attr", None) == "released"
                           for t in n.targets)]
        lines = SRC.splitlines()
        for ln in assigns:
            near = "\n".join(lines[max(0, ln - 6):ln])
            assert "_sigma_gate_ok" in near, (
                f"第 {ln} 行的放行沒有經過航向閘門 —— 實驗樣本會從這裡漏掉")


class TestTheGeometryThatMakesTheExperimentClean:
    """⭐ 實驗成立的前提是幾何:兩個切點的距離必須相同。這條把它釘住。"""

    D, A = 6147.0, 1300.0

    def _orbit(self, deg):
        th = math.radians(deg)
        px, py = self.A * math.cos(th), self.A * math.sin(th)
        rx, ry = self.D - px, -py
        vx, vy = -math.sin(th), math.cos(th)
        R = math.hypot(rx, ry)
        sig = abs(math.degrees(math.atan2(vx * ry - vy * rx, vx * rx + vy * ry)))
        return R, sig

    def test_both_tangent_points_are_at_the_same_range(self):
        tang = math.sqrt(self.D ** 2 - self.A ** 2)
        near = min(range(360), key=lambda d: self._orbit(d)[1])           # σ 最小
        far = min(range(360), key=lambda d: abs(self._orbit(d)[1] - 180))  # σ 最大
        Rn, sn = self._orbit(near)
        Rf, sf = self._orbit(far)
        assert abs(Rn - Rf) < 60.0, (
            f"兩個切點距離 {Rn:.0f} vs {Rf:.0f} m 差太多 —— 實驗的距離就沒被控制住,"
            f"共線問題會原封不動回來")
        assert abs(Rn - tang) < 60.0 and sn < 10.0 and sf > 170.0
        assert abs(Rn - 6058) < 60.0, "幾何變了,實驗設計要重算"

    @pytest.mark.parametrize("tol", [3, 5, 8, 10, 15, 20])
    def test_the_two_sigma_groups_are_range_matched_by_construction(self, tol):
        """⭐⭐ 實驗成立的真正理由(我第一版講錯了,測試抓到)。

        我原本寫「兩個切點距離相同 ⇒ 距離被控制住」,並斷言 2×2 幾何上取不到。
        測試打臉:±20° 容差下 σ<20° 的距離其實跨 887 m,不是「相同」。

        正確的說法是**分佈匹配**:因為兩個切點對稱於 D→T 連線,
        「σ≈0 的相位集合」與「σ≈180 的相位集合」對應到**同一組距離值**,
        兩群的距離中心**逐位元重合**(任何容差下都是 0 m 差)。
        ⇒ 共線不是靠「距離固定」打破的,是靠**兩群距離分佈相同**打破的。
        這比原本的說法弱一點,但仍然足以支持因果推論。
        """
        S = [self._orbit(d / 10.0) for d in range(3600)]
        a = [R for R, s in S if s <= tol]
        b = [R for R, s in S if abs(s - 180) <= tol]
        assert a and b, f"容差 {tol}° 取不到相位"
        ca, cb = (min(a) + max(a)) / 2, (min(b) + max(b)) / 2
        assert abs(ca - cb) < 5.0, (
            f"兩群距離中心差 {abs(ca - cb):.0f} m(容差 {tol}°)—— "
            f"若這個差變大,實驗就重新引入了距離共線,結論不能用")

    def test_the_chosen_tolerance_keeps_range_spread_small(self):
        """⚠️ 容差越鬆,群內距離跨度越大。選 ±8° 是等待時間與距離控制的折衷。"""
        S = [self._orbit(d / 10.0) for d in range(3600)]
        a = [R for R, s in S if s <= 8.0]
        spread = max(a) - min(a)
        assert spread < 450.0, (
            f"±8° 的群內距離跨度 {spread:.0f} m 太大。"
            f"⚠️ 對照:全 27 架次的 r(距離, 耗時) ≈ −0.10,"
            f"所以幾百公尺的殘餘變異對耗時的貢獻可忽略 —— 但別再放寬了。")
