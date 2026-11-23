# -*- coding: utf-8 -*-
"""S1:抵環即放行 + 內切即翻相的迴歸鎖(2026-08-18)。

═══ 為什麼要改 ═══

RRB 七趟 14 架次(⚠️ **不是八趟** —— RRB7 只有 contract json、沒有 CSV)實測:

| | 抵環當拍 | 放行當拍 |
|---|---|---|
| 前置角 σ | **0.1°**(14/14 落在 0.0–0.3°) | **130.2°**(最小 73.7°) |
| 距紅心 | 6079 m | **3417 m** |
| 高度 | — | 比抵環低 **970 m** |
| 高度散布 | 257 m | **805 m** |

中間隔了 **32.1 s**,而那 32 秒把上游辛苦收斂的東西全部吃回去。

⭐⭐ **σ≈0 的放行在全部語料裡從來沒有被取樣過**(最小值 73.7°)。
   純 PN 幾何(定速 150、R0=6084、N=3、4 g)算得出來為什麼這很重要:

       σ=0   耗時 40.4 s  d耗時/dσ 0.008 s/°  ⇒ 2 s 預算容許 **±125°**
       σ=30  耗時 41.5 s              0.084   ⇒ ±12°
       σ=90  耗時 53.0 s              0.376   ⇒ ±2.7°
       σ=125 耗時 72.8 s              0.952   ⇒ **±1.1°**

   ⇒ σ 不是「越小越好」的連續量,是**容忍度會塌掉**的量。
     抵環即放行的價值不是縮短耗時,是把系統移到對誤差不敏感的工作點。

放行距離同理:σ=0 時耗時 ≈ R/V,**ΔR 500 m ⇒ Δ耗時 3.3 s**。
RRB 放行 R 橫跨 2333–5560 m ⇒ 光這一項就值 ~21 s 的耗時散布。
抵環即放行把 R 釘在 ring_of(i),兩個變數一起解決。

⚠️ 這一批**只回答一個問題**:σ≈0 放行時,俯衝航跡長 / 放行直線距 會不會 ≤1.3?
   基準 14 架次是 1.60–4.49(中位 2.33)。過了才值得動架構;沒過(σ 已 ≤30°
   但比值仍 ≥1.6)⇒ 迂迴來自 PN 側向加速度飽和,整條路線要停下來改查 PN。
"""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))
UP = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(encoding="utf-8")
TABLE = (ROOT / "scripts" / "wsl" / "sar_swarm_patches"
         / "contract_table.py").read_text(encoding="utf-8")

SWITCHES = ("release_at_ring", "dive_from_runup")


def _code_only(block):
    """剝掉註解行與行尾註解。

    ⚠️ 「某字串**不可以**出現」的斷言一律要先剝註解 —— 本 session 已經栽過
       四次,每次命中的都是正在解釋舊行為的那句註解。
    """
    out = []
    for line in block.splitlines():
        if line.strip().startswith("#"):
            continue
        out.append(line.split("  #")[0] if "  #" in line else line)
    return "\n".join(out)


def _release_block():
    """取出放行判斷式那一段(elif 起,到 `elif self.dive_angle_max` 之前)。"""
    i = SRC.find("elif (self.release_at_ring and d <= self.ring_of(i))")
    assert i != -1, "⚠️ 抵環即放行的判斷式不見了"
    j = SRC.find("elif self.dive_angle_max > 0.0:", i)
    assert j != -1, "⚠️ 幾何守門員分支不見了 —— 它是 n=8 唯一的『打不到』出口"
    return _code_only(SRC[i:j])


class TestBothSwitchesDefaultOff:
    """⭐ 預設關閉 = 逐位元舊行為 = 一次 git checkout 回得去。"""

    @pytest.mark.parametrize("name", SWITCHES)
    def test_declared_default_is_false(self, name):
        for n in ast.walk(TREE):
            if (isinstance(n, ast.Call)
                    and (getattr(n.func, "id", None)
                         or getattr(n.func, "attr", None)) in ("g", "declare_parameter")
                    and len(n.args) >= 2
                    and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == name):
                assert n.args[1].value is False, f"{name} 的宣告預設必須是 False"
                return
        pytest.fail(f"找不到 {name} 的宣告")

    @pytest.mark.parametrize("name,env", [("release_at_ring", "RELEASE_AT_RING"),
                                          ("dive_from_runup", "DIVE_FROM_RUNUP")])
    def test_shell_default_is_false_and_not_empty(self, name, env):
        assert f'{env}:-False' in UP, (
            f"⚠️ {env} 的預設不是 False —— S1 會混進別的實驗")
        assert f'{env}:-}}' not in UP and f'{env}:-"' not in UP, (
            f"⚠️⚠️ {env} 預設成空字串 ⇒ `-p {name}:=` ⇒ rcl 解析失敗 ⇒ 節點起不來,"
            f"而症狀長得像埠號被搶走(RRA 整批 8 趟 / 5 小時就是這樣報廢的)")

    @pytest.mark.parametrize("name", SWITCHES)
    def test_it_is_wired_to_the_node(self, name):
        assert f'-p {name}:="$TOT_' in UP, f"⚠️ {name} 沒有接到節點上"

    @pytest.mark.parametrize("name", SWITCHES)
    def test_it_is_in_the_contract_table(self, name):
        assert f'"{name}", SOURCE_NODE' in TABLE, (
            f"⚠️⚠️ {name} 不在契約表 ⇒ 生效值 log 印不出來 ⇒ 事後無法證明它有沒有生效。\n"
            f"    前三批白跑有兩批(RRA/RRB)的症狀就是這個形狀。")
        assert f'"{name}"' in TABLE.split("BOOL_PARAMS")[1][:400], (
            f"⚠️ {name} 沒登記進 BOOL_PARAMS ⇒ 重跑提示會印成 `{name}:=1.0`,"
            f"餵給宣告為 BOOL 的參數 ⇒ InvalidParameterTypeException ⇒ 節點起不來")


class TestReleaseIgnoresTheSchedule:
    """⭐⭐ 這是 S1 的核心:開關開著時**完全不讀 dive_T**。"""

    def test_the_switch_short_circuits_before_tgo(self):
        """⚠️⚠️ 順序很重要:`self.release_at_ring and ...` 必須在左邊。

        時刻表本身不可達 —— [[predictor-underestimates-dive]] 記載預測器低估
        俯衝耗時中位 +18.7 s、**27/27 單邊為正**。把 `_tgo_of` 放左邊會讓
        它照樣被計算、照樣更新快取、照樣把壞掉的時基傳播下去。
        """
        b = _release_block()
        head = b.split("\n")[0] + "\n" + b.split("\n")[1] + "\n" + b.split("\n")[2]
        assert "self.release_at_ring and d <= self.ring_of(i)" in head, (
            "⚠️⚠️ 抵環判準不在第一個短路位置 —— dive_T 仍會被讀到")
        i_sw = b.find("self.release_at_ring")
        i_tgo = b.find("_tgo_of")
        assert 0 <= i_sw < i_tgo, (
            "⚠️⚠️ `_tgo_of` 出現在開關之前 ⇒ 短路失效,時刻表照樣參與放行")
        assert "not self.release_at_ring" in b, (
            "⚠️ 舊路徑沒有被 `not self.release_at_ring` 守住 ⇒ 兩條路徑會同時成立")

    def test_it_uses_the_strict_ring_not_the_gate_fraction(self):
        """⚠️ ring_gate_frac(1.10 ⇒ 6692 m)同時是四個機制對「環在哪裡」的共同
        認知:`_ring_transit_s`、`_tgo_of` 的分段點、`ring_pace` 的 eta 閘門、
        以及 runin/wait 的 far 判定。為了放行去改它會讓四者分裂。
        """
        b = _release_block()
        assert "d <= self.ring_of(i)" in b, "⚠️ 放行判準不是嚴格抵環"
        assert "ring_gate_frac" not in b, (
            "⚠️⚠️ 放行判準用了 ring_gate_frac —— 那會在 6692 m 就放行(環是 6084),"
            "而且動到它會讓另外四個機制對環的認知分裂")

    def test_the_sigma_gate_still_guards_the_release(self):
        """⚗️ 實驗閘門必須掛在**每一條**放行路徑上,否則對照是髒的。"""
        b = _release_block()
        assert "self._sigma_gate_ok(i)" in b, "⚠️ 新放行路徑沒有掛 σ 閘門"
        k = b.find("self.released[i] = True")
        assert k != -1 and "_sigma_gate_ok" in b[:k], (
            "⚠️ σ 閘門必須在 released 之前")

    def test_the_geometric_guard_is_still_reachable(self):
        """⚠️ 不可以順手刪 dive_angle_max:n=8 時它是唯一「為了對時間把自己
        等到打不到」的出口(AB1 的 UAV3 放行時需要 59.7° 而實得 35.3°)。"""
        assert "elif self.dive_angle_max > 0.0:" in SRC


class TestDiveFromRunupKeepsTheParameterWindow:
    """⚠️⚠️ 這一組鎖的是一個**完全靜默**的失效。

    `ring_t0 is not None and phase == "PERCH"` 這個視窗裡每 tick 重送
    PTCH_LIM_MIN_DEG=-80 / TECS_PITCH_MIN / TECS_SPDWEIGHT / THR_MAX /
    TECS_HDEM_TCONST,而原始碼自己的註解寫著「送出去了沒吃進去是已知會發生的」。

    視窗一塌成 1 個 tick,那組開關就等於沒開 ——
    而 [[steep-dive-switches-are-the-fix]] 記載它們是 0/6 變 4/6 的關鍵,
    且失效時 log 一切正常。
    """

    def test_ring_t0_starts_at_runup_not_at_arrival(self):
        i = SRC.find("if self.dive_from_runup:")
        assert i != -1, "⚠️ dive_from_runup 的分支不見了"
        b = _code_only(SRC[i:i + 700])
        assert "self.ring_t0 = self.runup_t0" in b, (
            "⚠️⚠️ ring_t0 沒有設成 runup_t0。設成 `now` 會讓倒數從**抵環**才開始 ——\n"
            "    那正是要移除的那 32 秒,飛機照樣往內漂 3 km、σ 照樣被甩到 130°。")

    def test_the_band_shortcut_is_disabled_when_the_switch_is_on(self):
        i = SRC.find("band_ok = ")
        assert i != -1, "⚠️ band 捷徑的守衛不見了"
        b = _code_only(SRC[i:i + 260])
        assert "not self.dive_from_runup" in b, (
            "⚠️⚠️ band 捷徑沒有被停用。dive_from_runup 開著時 ring_t0 從內切起點\n"
            "    計時,而那當下全隊剛爬完、高度可能本來就在 perch_band 內 ⇒\n"
            "    phase 在**一個 tick** 內翻掉 ⇒ 陡俯角參數的重送視窗塌成 1 次。\n"
            "    這個失效完全無聲:PTCH_LIM_MIN_DEG 會留在 .parm 的 −20,\n"
            "    log 一切正常,而 PN 要不到陡俯角。")
        assert "waited >= self.perch_settle" in b, (
            "⚠️ 逾時出口不見了 ⇒ phase 永遠不翻 ⇒ 一架都放不出去")

    def test_the_old_path_is_untouched_when_the_switch_is_off(self):
        """反向鎖:開關關著時必須還是「全員抵環才開始倒數」。"""
        assert "elif len(self.perch_arr) == self.n and self.ring_t0 is None:" in SRC, (
            "⚠️ 舊的『全員抵環』路徑被刪了 ⇒ 關閉時就不是逐位元舊行為")


class TestItScalesToEight:
    """⚠️ 目標規模 8 架 × 2 群 = 16 台。"""

    def test_the_reason_is_recorded_in_the_code(self):
        # ⚠️ 這一條**刻意**看原始文字(含註解)—— 它驗的就是理由有沒有寫下來。
        i = SRC.find("elif (self.release_at_ring and d <= self.ring_of(i))")
        j = SRC.find("elif self.dive_angle_max > 0.0:", i)
        b = SRC[i:j]
        assert "n=8" in b, "⚠️ 這段沒有記錄 n=8 的檢查"
        assert "_reground" in b, (
            "⚠️⚠️ 純幾何放行**沒有第二道防線**:`_reground()` 會在飛行中改寫\n"
            "    pushover_r ⇒ ring_of(i) 會跳 ⇒ 環一縮就全隊同一拍放行、\n"
            "    環一漲就把已在環內的機推回環外。海上目標最容易觸發。\n"
            "    這個風險必須留在程式碼裡。")
