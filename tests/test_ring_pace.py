# -*- coding: utf-8 -*-
"""V1:用速度取代環上盤旋等待的迴歸鎖(2026-08-16)。

═══ 為什麼要改 ═══

兩架的時刻差(實測,n=11):

    30 km  10.3 s      10 km  **4.4 s**      6.76 km  **40.2 s**

**上游同步得很好,最後 3.24 km 放大 9 倍。** 而放大的機制是盤旋等待:

| | 耗時 | 航跡長 | 迂迴 |
|---|---|---|---|
| 直線穿過 | 21–25 s | 3176–3224 m | **0.99×** |
| 盤旋等待 | 114–200 s | 9621–15332 m | **3.0–4.7×** |

⇒ 盤旋是用**成本 100+ 秒**的機制去吸收 **4.4 秒**的誤差,
   而吸收的過程本身製造了 40 秒的新誤差。
   更糟:從盤旋切出的相位不受控 ⇒ 放行航向 σ 隨機(實測中位 **151°**)。

⇒ 改成直線飛向環點(`goto_radius` 不是 `hold_radius`)+ 速度調節抵達時刻。
   3.24 km 在 95–157 m/s 下給 **13.5 s** 吸收力,需求 4.4 s ⇒ 餘裕 3 倍。

⚠️ **n=8 判準**:速度調節**每架獨立**、不搶共用資源 ⇒ 天然 scale。
   盤旋在 n=8 是七架擠同一個環(周長 8.2 km、迴轉半徑 2294 m)⇒ 空間放不下。
"""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))
UP = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(encoding="utf-8")


def _pace_block():
    """取出 `elif self.ring_pace ...` 到同層 `else:` 之間的那一段。

    ⚠️ 邊界必須用「換行 + **恰好** 20 個空格 + else:」——
       直接找 `"                    else:"` 會被內層(24 空格)的 `else:` 命中,
       因為 24 空格的那行**包含**20 空格的子字串。
       第一版就是這樣把區塊切掉一半,害 goto_radius 那條鎖誤報。
    """
    i = SRC.find("elif self.ring_pace and not far:")
    assert i != -1, "⚠️ V1 的速度調節分支不見了"
    j = SRC.find("\n" + " " * 20 + "else:", i)
    assert j != -1, "找不到同層的 else:"
    return _code_only(SRC[i:j])


def _code_only(block):
    """把註解行剝掉,只留程式碼。

    ⚠️⚠️ 這是本 session **第四次**踩同一個坑:用字串比對驗程式碼,結果命中的是
       **註解**。前三次分別是 `swarm_takeoff.sh` 的裸檔名、`self.pn_dt / 0.7`
       出現在註解裡、`weave_time_sync` 出現在 docstring 裡。
       本檔第四次:斷言「區塊裡不可以有 hold_radius」,而註解裡正好在解釋
       「舊行為用 hold_radius 繞圈」⇒ 誤報。
    ⇒ 凡是「某字串**不可以**出現」的斷言,都必須先剝註解。
      (「必須出現」的方向沒這個問題,但一起剝比較不會記錯。)
    """
    out = []
    for line in block.splitlines():
        s = line.strip()
        if s.startswith("#"):
            continue
        out.append(line.split("  #")[0] if "  #" in line else line)
    return "\n".join(out)


class TestTheSwitchDefaultsOff:
    """⭐ 預設關閉 = 逐位元舊行為。⚠️ 一次只動一個變數是這輪的紀律。"""

    def test_declared_default_is_false(self):
        for n in ast.walk(TREE):
            if (isinstance(n, ast.Call)
                    and (getattr(n.func, "id", None) or
                         getattr(n.func, "attr", None)) in ("g", "declare_parameter")
                    and len(n.args) >= 2
                    and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == "ring_pace"):
                assert n.args[1].value is False, "ring_pace 宣告預設必須是 False"
                return
        pytest.fail("找不到 ring_pace 的宣告")

    def test_shell_default_is_off_too(self):
        assert 'RING_PACE:-False' in UP, (
            "⚠️ 啟動腳本的 ring_pace 預設不是 False —— V1 會混進別的實驗")

    def test_it_is_wired_to_the_node(self):
        assert '-p ring_pace:="$TOT_RING_PACE"' in UP


class TestItFliesToTheRingInsteadOfOrbiting:
    """⭐⭐ 這是整個 V1 的核心:半徑參數決定「飛過去」還是「繞圈」。"""

    def test_uses_goto_radius_not_hold_radius(self):
        b = _pace_block()
        assert "radius=self.goto_radius" in b, (
            "⚠️⚠️ 速度調節分支沒有用 goto_radius —— 那飛機還是在繞圈,V1 等於沒做。\n"
            "    goto_radius = 飛過去;hold_radius = 繞著它轉。")
        assert "hold_radius" not in b, (
            "⚠️⚠️ 速度調節分支裡出現 hold_radius —— 那就是盤旋,正是要移除的東西")

    def test_the_old_orbit_branch_still_exists_for_the_off_path(self):
        """反向鎖:開關關著時必須還是舊行為。"""
        # ⚠️ 不要用「從分支起點數 N 個字元」當視窗 —— 註解一長就掃不到,
        #    這條鎖就會誤報成「舊分支被刪了」(第一版 4000 字元正是這樣爆的)。
        #    改成從同層 else: 起算,邊界跟 _pace_block() 用同一條規則。
        i = SRC.find("elif self.ring_pace and not far:")
        j = SRC.find("\n" + " " * 20 + "else:", i)
        # ⚠️ 視窗長度是這條鎖第**二**次的失敗來源(第一次是 4000、這次是 1500):
        #    註解一長就掃不到,誤報成「舊分支被刪了」。
        #    ⇒ 不要數字元 —— 掃到下一個同層 `stat[i] =` 為止,那是分支的真結尾。
        end = SRC.find("\n" + " " * 20 + "stat[i] = ", j)
        assert end != -1, "找不到分支結尾"
        tail = SRC[j:end]
        # ⚠️ 2026-08-18:S2 在同一個運算式裡加了 `or self.wait_goto_radius`,
        #    所以錨點從 "if far" 變成 "if (far or self.wait_goto_radius)"。
        #    這條鎖要驗的東西沒有變:**兩顆開關都關時仍然退回 hold_radius**。
        assert "radius=(self.goto_radius" in tail, (
            "⚠️ 舊的盤旋分支被刪了 —— 開關關閉時就不是逐位元舊行為了")
        assert "else self.hold_radius))" in tail, (
            "⚠️⚠️ 關閉路徑不再退回 hold_radius ⇒ 開關關著時就不是逐位元舊行為,"
            "對照組會被污染")
        assert "far or self.wait_goto_radius" in tail, (
            "⚠️ S2 的 wait_goto_radius 不在這個運算式裡 —— 它應該與 far 同權")


class TestSpeedIsComputedFromSlack:
    """⭐ 速度必須由「還有多少時間」與「還有多遠」算出來,不是常數。"""

    def test_speed_uses_a_relative_reference_not_dive_T(self):
        """⭐⭐ RRB(2026-08-16)整批 8 趟白跑的教訓。

        第一版用 `slack = (dive_T − now) − t_go` 當參考。但**飛機永遠是遲到的**
        (memory/predictor-underestimates-dive:27/27 單邊,e_s 從放行第一拍就 ≤0)
        ⇒ `slack > 0.5` 幾乎不成立 ⇒ 每次掉到「全速趕」⇒ v_cmd 恆等於 vmax,
        十四架次無一例外。**分支開著,但邏輯從未執行。**

        ⚠️ 這與 ITCG 從未武裝是**同一個根因**。⇒ 參考量必須與 dive_T 無關。
        """
        b = _pace_block()
        assert "dive_T" not in b, (
            "⚠️⚠️ 速度調節又用回 dive_T 了。時刻表本身就不可達"
            "(預測器低估俯衝耗時中位 +18.7 s)⇒ 裕度恆為負 ⇒ 分支永遠不執行。")
        assert "t_ring" in b and "d_go" in b, "沒有用到相對抵環時刻與剩餘距離"
        assert "d_go / t_ring" in b, (
            "⚠️ 速度不是由 距離/時間 算的 —— 那就不是在調節抵達時刻")
        assert "max(eta)" in b, (
            "⚠️ 目標時刻必須取全隊的**最大**(最慢那架)——"
            "取平均會讓最慢的追不上,取最小則全隊一起早到")

    def test_speed_is_clamped_to_the_envelope(self):
        b = _pace_block()
        assert "self.vmin" in b and "self.vmax" in b, (
            "⚠️⚠️ 速度沒有夾在包絡內。低於 vmin 會被飛控整發拒收"
            "(catch-speed-rejected-by-fc:MAV_RESULT=4,而且完全靜默)")
        assert "max(self.vmin" in b and "min(self.vmax" in b

    def test_no_division_by_zero(self):
        b = _pace_block()
        assert "t_ring > 0.5" in b and "d_go > 1.0" in b, (
            "⚠️ 除法沒有守衛 —— t_ring→0 會讓速度指令爆掉")

    def test_the_slowest_aircraft_is_not_slowed_further(self):
        """⭐ 取 max ⇒ 最遠那架的 d_go/t_ring 就等於 vmax ⇒ 不被拖慢;
        其餘的 d_go 較小 ⇒ 速度較低 ⇒ 放慢等它。這是整個機制的方向性。"""
        b = _pace_block()
        assert "for q in self.pos" in b, (
            "⚠️ 沒有對全隊取 ETA —— 那就不是相對參考,又變成各飛各的")

    def test_eta_denominator_is_capability_not_measured_speed(self):
        """⚠️⚠️ ETA 的分母必須是 vmax(能力),不能是實測地速 self.gs。

        用地速 ⇒ **正回饋棘輪**:最遠那架算出的
            v = d_max / t_ring,而 t_ring ≥ d_max / gs_自己
        ⇒ v ≤ 自己現在的速度 ⇒ 下一拍更慢 ⇒ t_ring 更大 ⇒ 再更慢,
        全隊一路滑到 vmin。而「速度釘在 vmin」正是
        [[dive-pace-saturates-vmin]] 記錄過、同時毀掉散布**與**終端速度的坑。

        ⇒ 分母用 vmax:t_ring = max_j(d_j)/vmax,只跟幾何有關,沒有回饋,
          而且最遠那架恰好拿到 vmax。
        """
        b = _pace_block()
        assert "self.gs" not in b, (
            "⚠️⚠️ ETA 用了實測地速 —— 正回饋,全隊會棘輪到 vmin。分母請用 self.vmax")
        assert "/ self.vmax" in b, "⚠️ ETA 的分母不是 vmax"

    def test_each_aircraft_uses_its_own_ring_radius(self):
        """⚠️ perch_step 開著時每架環半徑不同(ring_of(i) 依 perch_of(i) 算)。
        拿本機的閘門距離去算別人的剩餘距離 ⇒ 系統性偏差,而且隨隊形放大。"""
        b = _pace_block()
        assert "self.ring_of(q)" in b, (
            "⚠️ 別人的閘門距離用了本機的環半徑 —— perch_step 開著時每架不同")


class TestItScalesToEight:
    """⚠️ 目標規模 8 架 × 2 群 = 16 台。任何「共用資源」的機制都要先過這一關。"""

    def test_the_comment_records_the_n8_check(self):
        # ⚠️ 這一條**刻意**看原始文字(含註解)—— 它驗的就是「理由有沒有寫下來」。
        i = SRC.find("elif self.ring_pace and not far:")
        j = SRC.find("\n" + " " * 20 + "else:", i)
        b = SRC[i:j]
        # ⚠️ 第一版只驗 "n=8" 有沒有出現 —— 變異測試證明那太軟(區塊裡出現兩次,
        #    刪一處另一處還在)。改成驗**真正承重的那句理由**。
        assert "n=8" in b, "⚠️ 這段沒有記錄 n=8 的檢查"
        assert "每架獨立" in b and "共用資源" in b, (
            "⚠️⚠️ 速度調節之所以對,理由是它**每架獨立、不搶共用資源**;\n"
            "    盤旋之所以錯,理由是 n=8 時七架擠同一個環(周長 8.2 km、\n"
            "    迴轉半徑 2294 m)⇒ 空間放不下。\n"
            "    這個理由必須留在程式碼裡 —— 否則下一個人看到「盤旋比較穩」\n"
            "    就會把它改回去,而 n=2 的測試不會抓到。")
