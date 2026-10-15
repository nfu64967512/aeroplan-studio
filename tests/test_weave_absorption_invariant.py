# -*- coding: utf-8 -*-
"""`_inv_weave_period_can_actually_absorb_time` 的值層級鎖(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這條不變式在守什麼
═══════════════════════════════════════════════════════════════════════════
蛇形能吸收多少時間 = 介入窗(秒) × 實得路徑拉長比。

    實得拉長 = (1/cos θ − 1) × 達成率
    θ        = min(weave_theta_max, ω·P/8)      ω = g·tanφ/V
    介入窗   = (環半徑 × 0.4) / V               環 = perch_alt / tan(dive_angle)

低於 WEAVE_MIN_USEFUL_S 就是裝飾品 —— 而裝飾品會被當成「試過了、沒用」
寫進結論。

⚠️⚠️ **本條的第一版把吸收量高估四倍。** 它只看換向率 θ ≤ ω·P/8,宣稱
   P=120 能吸「41.4% ≈ 12 s」。WX1-3(2026-08-23)實測是 **2.85 s**,因為:
     * 介入窗只有 22.9 s,而半週期是 60 s ⇒ **一次換向都沒有**(0.21 個週期);
     * 指令因此退化成單邊斜飛,實得/指令迂迴只有 **32% / 33% / 24%**。

═══════════════════════════════════════════════════════════════════════════
 ⚠️⚠️ 為什麼測試點是俯角 54° 而不是生產的 20°
═══════════════════════════════════════════════════════════════════════════
2026-08-23 的刪除實驗:在生產組態(俯角 20°、P=120 vs P=20)上,**四個突變
裡有三個完全不改變判定** ——

    拿掉達成率(當成 100% 達成)      判定不變
    窗長改用 vmin(高估 1.58 倍)     判定不變
    拿掉 weave_theta_max 夾限        判定不變

因為在那兩個點上,判定被「P=20 時 θ 被換向率壓到 9.4°」單獨決定,其餘因子
再怎麼變都跨不過門檻。⇒ 那三個因子當時**一條測試都沒有**,可以整個寫錯
而全綠。

能鑑別它們的是**判定臨界附近**的組態:介入窗約 6 秒(俯角 54° ⇒ 環 2252 m
⇒ 窗 6.0 s),此時吸收量 0.75 s 剛好落在 1.0 s 門檻之下,而上面任何一個因子
被移除都會把它推上去 ⇒ 判定翻面。本檔的核心案例就是那一點。

⭐ 一般教訓:守衛要測在**它會改變答案**的地方,不是測在最常用的地方。
"""
import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "wsl" / "sar_swarm_patches"))

import contract_table as ct                                    # noqa: E402

INV = ct._inv_weave_period_can_actually_absorb_time

#: 生產組態(WX1-3 實際飛的)。
PROD = {"weave_time_sync": 1.0, "weave_period": 120.0, "vmax": 150.0,
        "perch_alt": 3100.0, "dive_angle": 20.0, "weave_theta_max": 45.0}

#: ⭐ 判定臨界點:俯角 54° ⇒ 環 2252 m ⇒ 介入窗 6.0 s ⇒ 吸收 0.75 s(< 1.0)。
#:   三個因子任何一個被拿掉都會把它推過門檻 ⇒ 這是唯一測得到它們的地方。
EDGE = dict(PROD, dive_angle=54.0)


def check(**over):
    vals = dict(PROD, **over)
    return INV(lambda k: vals.get(k))


class TestTheProductionConfigIsAllowedButNotFlattered:
    """⭐ WX1-3 的組態吸收 2.85 s —— 少於原本宣稱的 12 s,但**確實有用**。

    擋掉它是錯的:實測放行時刻差 2.4–15.2 s,2.85 s 治得了其中最小的那些。
    這條不變式的工作是讓人**知道**是 2.85 而不是 12,不是禁止使用。
    """

    def test_it_passes(self):
        assert check() is None

    def test_the_arithmetic_matches_what_was_flown(self):
        """獨立算一遍,確認不變式用的幾何與實測 wait 段對得上。"""
        ring = PROD["perch_alt"] / math.tan(math.radians(PROD["dive_angle"]))
        window = ring * 0.4 / PROD["vmax"]
        assert window == pytest.approx(22.7, abs=0.5), (
            f"介入窗算出 {window:.1f}s,而 WX1-3 實測 wait 段是 21–27 s")
        om = math.degrees(ct.G * math.tan(math.radians(ct.BANK_DEG)) / PROD["vmax"])
        th = min(PROD["weave_theta_max"], om * PROD["weave_period"] / 8.0)
        assert th == pytest.approx(45.0), "θ 應該被 weave_theta_max 咬住,不是被換向率"
        absorb = window * (1 / math.cos(math.radians(th)) - 1) * ct.WEAVE_ACHIEVED_FRAC
        assert absorb == pytest.approx(2.8, abs=0.3), (
            f"吸收量算出 {absorb:.2f}s;原 docstring 宣稱 12 s(高估四倍)")


class TestTheShortPeriodHornIsCaught:
    """⚠️ 另一隻角:週期短到塞得進窗,但換向率把 θ 壓死。"""

    @pytest.mark.parametrize("per,want_s", [(20.0, 0.09), (10.0, 0.02)])
    def test_a_period_that_fits_the_window_absorbs_nothing(self, per, want_s):
        v = check(weave_period=per)
        assert v is not None, f"P={per} 只吸收得到 {want_s} s,必須擋"
        assert f"{want_s:.2f} s" in v.got, f"訊息要印出秒數,實得:{v.got}"

    def test_the_message_names_the_right_horn(self):
        """⭐ 兩隻角的修法不同:週期短 ⇒ 加大週期;窗短 ⇒ 拉長介入帶。
        訊息說錯會把人送去改錯的旋鈕。"""
        assert "換向率把 θ 壓死" in check(weave_period=20.0).why
        assert "介入窗太短" in check(dive_angle=54.0, weave_period=120.0).why


class TestTheEdgeCaseThatDiscriminatesEveryFactor:
    """⭐⭐ 這一組是刪除實驗證明有效的唯一測試點,見檔頭。"""

    def test_the_edge_config_is_just_below_the_threshold(self):
        v = INV(lambda k: EDGE.get(k))
        assert v is not None, "俯角 54° 的窗只有 6 s,吸收 0.75 s ⇒ 必須擋"
        ring = EDGE["perch_alt"] / math.tan(math.radians(EDGE["dive_angle"]))
        window = ring * 0.4 / EDGE["vmax"]
        absorb = window * (1 / math.cos(math.radians(45.0)) - 1) * ct.WEAVE_ACHIEVED_FRAC
        assert window == pytest.approx(6.0, abs=0.3)
        assert 0.5 < absorb < ct.WEAVE_MIN_USEFUL_S, (
            f"吸收 {absorb:.2f}s 必須落在門檻**之下但不遠**,否則測不到那三個因子")

    def test_pretending_the_weave_is_fully_achieved_would_flip_it(self):
        """⚠️ 達成率是**實測**的 0.30(WX1-3:32/33/24%)。當成 1.0 的話
        吸收變 2.49 s ⇒ 這個組態會被放行,而它其實只做得到 0.75 s。"""
        ring = EDGE["perch_alt"] / math.tan(math.radians(EDGE["dive_angle"]))
        window = ring * 0.4 / EDGE["vmax"]
        naive = window * (1 / math.cos(math.radians(45.0)) - 1)      # 達成率 = 1.0
        assert naive > ct.WEAVE_MIN_USEFUL_S, (
            "若 100% 達成就會放行 ⇒ 達成率這個因子在本測試點確實有鑑別力")
        assert ct.WEAVE_ACHIEVED_FRAC == pytest.approx(0.30), (
            "達成率改了就要重測 —— 它直接決定這個組態擋不擋")

    def test_using_vmin_for_the_window_would_flip_it(self):
        """⚠️ 窗長必須用 **vmax** 估:飛得快 ⇒ 窗短 ⇒ 判定嚴。
        用 vmin(95)會把窗高估 1.58 倍 ⇒ 吸收 1.18 s ⇒ 放行。
        而實測守環段的**實得**地速遠高於指令值([[hold-ring-is-unflyable]]),
        所以高估的方向剛好是「看起來沒問題」。"""
        ring = EDGE["perch_alt"] / math.tan(math.radians(EDGE["dive_angle"]))
        with_vmin = ring * 0.4 / 95.0 * (
            1 / math.cos(math.radians(45.0)) - 1) * ct.WEAVE_ACHIEVED_FRAC
        assert with_vmin > ct.WEAVE_MIN_USEFUL_S, (
            "用 vmin 會放行 ⇒ 這個選擇在本測試點有鑑別力")

    def test_dropping_the_theta_ceiling_would_flip_it(self):
        """⚠️ P=120 時換向率允許 56.3°,而 weave_theta_max 把它壓到 45°。
        不夾的話拉長從 12.4% 變 24.1% ⇒ 吸收 1.44 s ⇒ 放行。"""
        om = math.degrees(ct.G * math.tan(math.radians(ct.BANK_DEG)) / EDGE["vmax"])
        th_free = om * EDGE["weave_period"] / 8.0
        assert th_free > EDGE["weave_theta_max"], "前提:換向率比 theta_max 寬鬆"
        ring = EDGE["perch_alt"] / math.tan(math.radians(EDGE["dive_angle"]))
        window = ring * 0.4 / EDGE["vmax"]
        unclamped = window * (
            1 / math.cos(math.radians(th_free)) - 1) * ct.WEAVE_ACHIEVED_FRAC
        assert unclamped > ct.WEAVE_MIN_USEFUL_S, (
            "不夾 θ 會放行 ⇒ theta_max 這個夾限在本測試點有鑑別力")


class TestItStaysQuietWhenItShould:
    def test_the_switch_off_is_not_judged(self):
        assert check(weave_time_sync=0.0) is None

    def test_missing_geometry_is_left_to_the_missing_check(self):
        """⚠️ 取不到幾何就**不猜**。猜一個環半徑會產生一個看起來合理的
        吸收量,而那正是這條不變式要防的形狀。"""
        assert check(perch_alt=None) is None
        assert check(dive_angle=None) is None
        assert check(dive_angle=0.0) is None


class TestTheThresholdHasAProvenance:
    """⭐ 門檻不可以是隨手填的數。"""

    def test_it_is_below_every_measured_release_gap(self):
        """實測放行時刻差 2.4–15.2 s(WX1-3)⇒ 連 1 秒都動不了的通道
        對其中**任何一趟**都無能為力。"""
        measured_gaps = (2.4, 7.6, 15.2)
        assert ct.WEAVE_MIN_USEFUL_S < min(measured_gaps), (
            "門檻若高於最小的實測差距,就會擋掉治得了它的組態")
        assert ct.WEAVE_MIN_USEFUL_S >= 0.5, (
            "門檻太低會把裝飾品當成通道 —— WX 前的 P=20 只吸 0.09 s")
