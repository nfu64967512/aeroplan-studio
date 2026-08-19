# -*- coding: utf-8 -*-
"""航跡角 γ 的符號約定鎖(2026-08-15)。

═══ 為什麼需要 ═══

`tot_node.py` 裡有**三個**都叫 `gam` 的量,而且分成**兩種相反約定**:

  ① `gate_r_of`  : `atan2(vz, gs)`,vz 是 MAVLink 的**下正** ⇒ 下降為 **正**
  ② `_pn_cmd`    : `atan2(hdot, vh)`,`hdot = −vd` 是**上正** ⇒ 下降為 **負**
  ③ CSV `gam_deg`: `atan2(−vz, …)` ⇒ 下降為 **負**(與②同)

三處各自內部自洽,**不可以順手統一** —— 改符號會直接翻轉 ① 的比較方向。
但引用任何一個 γ 之前必須先確認是哪一種。

⚠️ 已經出過事:CSV `gam_deg` 的欄位註解一度寫成「下降為正」(寫反),
   事後判讀時「最陡」會被讀成 `max()` 而實際上該取 `min()`,結論剛好顛倒。
   memory/confirm-direction-not-just-topic 講的就是這一類 ——
   **主題對不代表方向對**,而方向錯的註解比沒有註解更糟。
"""
import ast
import math
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")


def _csv_gamma(vn, ve, vd):
    """複製 `_row` 裡 `_gam` 的算式(③)。vd 為 MAVLink 下正。"""
    return math.degrees(math.atan2(-vd, max(math.hypot(vn, ve), 1e-6)))


class TestCsvGammaIsNegativeWhenDescending:
    """⭐ CSV 是**事後判讀**唯一的來源,它的方向錯了整批結論都會反。"""

    def test_descending_gives_negative(self):
        # 水平 150 m/s、下降 80 m/s(vd 下正)
        assert _csv_gamma(150.0, 0.0, 80.0) < 0.0

    def test_climbing_gives_positive(self):
        assert _csv_gamma(150.0, 0.0, -80.0) > 0.0

    def test_level_is_zero(self):
        assert abs(_csv_gamma(150.0, 0.0, 0.0)) < 1e-9

    def test_the_source_still_negates_vd(self):
        """結構鎖:算式一旦少了那個負號,方向就整個翻掉而數值看起來還很正常。"""
        assert "-_v[2], max(math.hypot(_v[0], _v[1]), 1e-6)" in SRC, (
            "⚠️⚠️ CSV 的 gam_deg 算式變了。確認方向約定仍是**下降為負**,"
            "並同步更新欄位註解與本測試。")

    def test_the_column_comment_states_the_direction_correctly(self):
        """⚠️ 註解必須說「下降為負」。曾經寫成「下降為正」——完全寫反。"""
        i = SRC.find("# gam_deg")
        assert i != -1, "找不到 gam_deg 的欄位註解"
        block = SRC[i:i + 400]
        assert "下降為負" in block, (
            "⚠️⚠️ gam_deg 的欄位註解沒有寫明**下降為負**。\n"
            "    方向寫錯或沒寫的註解會讓事後判讀把 max/min 取反,結論顛倒。")
        assert "下降為正" not in block.split("下降為負")[0], (
            "⚠️ 註解裡同時出現「下降為正」—— 兩種說法並存等於沒有約定。")


class TestTheTwoConventionsAreBothDocumented:
    """⭐⭐ 兩種約定並存是**刻意**的,但必須寫明,否則下一個人一定會統一它。"""

    def test_gate_r_of_marks_its_convention(self):
        i = SRC.find("gam = math.atan2(vz, max(gs, 1e-6))")
        assert i != -1, "gate_r_of 的 γ 算式不見了"
        near = SRC[max(0, i - 700):i + 120]
        assert "下降為正" in near, (
            "⚠️ gate_r_of 用的是**下正**的 vz(下降為正),但附近沒有寫明。")
        assert "兩種相反" in near, (
            "⚠️ 沒有提醒「本檔有兩種相反的 γ 約定」—— "
            "下一個人會把它與 _pn_cmd 的 γ 混用。")

    def test_pn_cmd_uses_the_up_positive_form(self):
        assert "hdot = -vd" in SRC, (
            "⚠️ `_geom` 的 hdot 不再是 −vd。_pn_cmd 的 γ 約定跟著翻轉,"
            "而 CSV 那一側不會自動跟著改。")
