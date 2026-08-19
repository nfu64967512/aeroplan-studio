# -*- coding: utf-8 -*-
"""導引層觀測性與生效速率的迴歸鎖(S1 / S2)。

═══ 為什麼這兩步排在整改的最前面 ═══

Hybrid Trajectory Shaping 要把姿態通道從終端段往前延伸,而在那之前有兩件事必須成立:

1. **看得見**:γ / α / θ_cmd / 誰在飛 / 飛控模式,現在全部只存在於節點記憶體裡。
   看不見就無法歸因,而本專案今天剛因為「三項一起改」而無法歸因一次。
2. **宣告值 = 生效值**:`pn_hz` 宣告 50 Hz,但 `_pn_cmd` 只從 20 Hz 的 `_tick`
   被呼叫 ⇒ α 濾波的時間常數宣稱 0.7 s、**實際 1.75 s**,慢 2.5 倍,
   而且 log 還印「就緒(50 Hz)」。三個數字互相矛盾了很久,沒有東西在比對。

⚠️⚠️ S2 是**恆等改寫**:`(1/50)/0.7 ≡ (1/20)/1.75`。若哪天這個恆等式不成立,
   代表有人改了其中一邊而沒改另一邊,飛行行為會**無聲**改變 —— 所以要鎖。
"""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))


def _module_const(name):
    for n in TREE.body:
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant):
            for t in n.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return n.value.value
    raise AssertionError(f"tot_node.py 找不到模組常數 {name}")


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
                return float(v.value)
    raise AssertionError(f"tot_node.py 找不到參數宣告 {name}")


class TestTickRateIsASingleSourceOfTruth:
    """⭐ 主迴圈週期只能有一個來源。

    ⚠️ 先前 `create_timer(0.05, …)` 寫死、參數 `pn_hz` 另外宣告 50 ——
       兩個數字各自獨立,沒有任何東西保證它們一致,而它們確實不一致。
    """

    def test_tick_period_constant_exists(self):
        assert _module_const("TICK_PERIOD_S") == pytest.approx(0.05)

    def test_the_timer_uses_the_constant_not_a_literal(self):
        """⚠️ 若有人把 create_timer 改回寫死數字,α 濾波就會再次無聲脫鉤。"""
        assert "create_timer(TICK_PERIOD_S, self._tick)" in SRC, \
            "主 timer 沒有引用 TICK_PERIOD_S —— 生效速率又變成寫死值了"

    def test_the_alpha_filter_uses_the_real_period(self):
        assert "self.tick_dt / self.pn_alpha_tau" in SRC, \
            "α 濾波增益沒有用實際週期 —— 又回到用宣告的 pn_hz"
        # ⚠️ 只看**非註解**行:舊寫法被刻意引用在說明註解裡當反例,
        #    直接對整份原始碼做 substring 檢查會被自己的文件觸發(踩過一次)。
        code = [ln.split("#", 1)[0] for ln in SRC.splitlines()]
        live = [ln for ln in code if "self.pn_dt / 0.7" in ln or "self.pn_dt/0.7" in ln]
        assert not live, f"舊的寫死 0.7 還活在程式碼裡:{live}"


class TestTheRewriteIsBitForBitIdentical:
    """⚠️⚠️ 這一條是 S2 敢動的唯一理由。

    舊式 k = (1/pn_hz)/0.7,新式 k = TICK_PERIOD_S/pn_alpha_tau。
    預設值下兩者必須**完全相同**,否則這不是恆等改寫而是行為改動,
    而行為改動不可以夾在「修可觀測性」裡偷渡。
    """

    def test_gains_match_exactly(self):
        old = (1.0 / _declared("pn_hz")) / 0.7
        new = _module_const("TICK_PERIOD_S") / _declared("pn_alpha_tau")
        assert abs(old - new) < 1e-12, (
            f"舊增益 {old!r} ≠ 新增益 {new!r} —— 這就不是恆等改寫了。\n"
            f"    pn_hz={_declared('pn_hz')}、pn_alpha_tau={_declared('pn_alpha_tau')}、"
            f"TICK_PERIOD_S={_module_const('TICK_PERIOD_S')}")

    def test_the_real_time_constant_is_documented(self):
        """實際 τ 是 1.75 s。這個數字要留在程式碼裡,不是只留在對話裡。"""
        assert _declared("pn_alpha_tau") == pytest.approx(1.75)

    def test_the_nominal_rate_is_marked_as_nominal(self):
        """⚠️ `pn_hz` 保留只為相容。註解必須講明它不是生效值,
        否則下一個人還是會拿它去調速率。"""
        i = SRC.index('g("pn_hz"')
        head = SRC[max(0, i - 700):i + 120]
        assert "生效值" in head or "名義" in head, \
            "pn_hz 附近沒有說明它是名義值而非生效值"


class TestContractLineSeesTheGuidanceSwitches:
    """契約層只能檢查**讀得到**的東西。

    ⚠️⚠️ `gate_theta_max` 先前不在契約行裡 ⇒ 「動態交棒有沒有開」對契約層不可見。
       而它 <= 0 時 `gate_of()` 的動態分支是死碼、直接回 `gate_r`,
       log 仍印「(絕對 gate_r)」—— 開與沒開長得一模一樣。
    """

    def _emitted(self):
        for node in ast.walk(TREE):
            if isinstance(node, ast.FunctionDef) and node.name == "_contract_values":
                return {e.elts[0].value for e in ast.walk(node)
                        if isinstance(e, ast.Tuple) and len(e.elts) == 2
                        and isinstance(e.elts[0], ast.Constant)
                        and isinstance(e.elts[0].value, str)}
        raise AssertionError("找不到 _contract_values()")

    REQUIRED = [
        "gate_theta_max", "gate_phi", "gate_hold_n",
        "tick_hz", "pn_alpha_tau", "pn_hz_nominal",
        "dive_pitch_min", "dive_pitch_max",
        "pn_lead", "pn_vmax", "line_tol", "perch_step",
        "pn_los_floor", "pn_nz_min",
    ]

    @pytest.mark.parametrize("key", REQUIRED)
    def test_key_is_emitted(self, key):
        assert key in self._emitted(), (
            f"{key} 沒印進契約行 ⇒ contract_verify 讀不到 ⇒ 一律判 missing,"
            f"而「全部讀不到」看起來像「節點沒起來」")

    def test_both_the_nominal_and_effective_rate_are_emitted(self):
        """⚠️ 兩個都要印。只印生效值的話,下次再脫鉤時看不出「宣告了什麼」。"""
        e = self._emitted()
        assert {"tick_hz", "pn_hz_nominal"} <= e

    def test_every_emitted_key_resolves_to_an_attribute(self):
        """⚠️ 印一個不存在的屬性 = 節點啟動時 AttributeError = 整趟起不來。
        這條在 Windows 上就能抓到,不必等到 WSL。"""
        emitted = self._emitted()
        for node in ast.walk(TREE):
            if isinstance(node, ast.FunctionDef) and node.name == "_contract_values":
                attrs = {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}
                break
        # 每個 self.X 都要在檔案某處被賦值
        for a in attrs:
            if a in ("get",):
                continue
            assert f"self.{a} =" in SRC or f"self.{a}=" in SRC, \
                f"_contract_values 引用了 self.{a},但全檔找不到它的賦值"
        assert len(emitted) >= 25, f"契約行只印了 {len(emitted)} 項,似乎掉了東西"
