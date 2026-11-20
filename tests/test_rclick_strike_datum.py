# -*- coding: utf-8 -*-
"""右鍵打擊的高度基準 —— 海上紅心非帶 amsl 不可。

═══ 2026-08-07 發現 ═══
`build_target_payload` 只送 `lat` / `lon` / `approach_from`,**沒有 `amsl`**。
於是 tot_node 的 `target_amsl` 保持 nan ⇒ 回頭去查地形庫,
而 **SRTM 在水域沒有資料、回的是垃圾**(本專案實測散布 17 m,門檻是 3 s)。

批次路徑早就用 `TARGET_AMSL=0.0` 釘死(repro_two1.sh),
右鍵這條**產品**路徑卻從來沒有這個欄位 —— 等於每一發海上打擊
都在用垃圾高程,而且完全靜默:座標對、topic 對、rc=0,只有落點是錯的。

⚠️ 刻意做成 Optional 而不是預設 0.0:陸上目標查地形高才是對的,
   預設任一邊都會在另一邊靜默地錯。由呼叫端(確認框的兩顆鈕)明示。
"""
import json
import pathlib
import re
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from integrations import rclick_strike as rcs        # noqa: E402

LAT, LON = 25.0777000, 120.7366400                   # 台灣海峽的紅心
CTRL = (ROOT / "ui" / "controllers" /
        "rclick_strike_controller.py").read_text(encoding="utf-8")


def _payload_of(argv):
    """從組好的 wsl.exe argv 裡把 JSON 解回來。"""
    return rcs.decode_publish_arg(" ".join(argv))


class TestPayloadCarriesAmsl:
    def test_sea_target_includes_amsl(self):
        got = json.loads(rcs.build_target_payload(LAT, LON, amsl=0.0))
        assert got["amsl"] == pytest.approx(0.0)

    def test_land_target_omits_amsl(self):
        """省略 = 「請查地形庫」,和 amsl=0 是**不同**語意,不可以混。"""
        got = json.loads(rcs.build_target_payload(LAT, LON))
        assert "amsl" not in got

    def test_non_zero_amsl_supported(self):
        """山區紅心也可能要釘死(地形快取不可信時)。"""
        got = json.loads(rcs.build_target_payload(LAT, LON, amsl=250.0))
        assert got["amsl"] == pytest.approx(250.0)

    def test_nan_amsl_is_rejected_loudly(self):
        """⚠️ json.dumps(nan) 產生 `NaN` —— **不是合法 JSON**,
        節點端 json.loads 會整包丟掉 ⇒ 整發打擊靜默消失。"""
        with pytest.raises(rcs.StotBridgeError):
            rcs.build_target_payload(LAT, LON, amsl=float("nan"))
        with pytest.raises(rcs.StotBridgeError):
            rcs.build_target_payload(LAT, LON, amsl=float("inf"))

    def test_payload_is_always_valid_json(self):
        for amsl in (None, 0.0, -3.5, 1200.0):
            json.loads(rcs.build_target_payload(LAT, LON, amsl=amsl,
                                                approach_from=90.0))

    def test_amsl_is_an_optional_key(self):
        assert "amsl" in rcs.PAYLOAD_OPTIONAL_KEYS
        assert "amsl" not in rcs.PAYLOAD_REQUIRED_KEYS

    def test_coords_still_verbatim(self):
        """既有合約不動:%.7f 逐字,不走 json.dumps 的最短表示。"""
        assert '"lat": 25.0777000' in rcs.build_target_payload(LAT, LON, amsl=0.0)


class TestCommandRoundTrip:
    def test_amsl_survives_the_three_layers_of_quoting(self):
        """bash 雙引號 → YAML 單引號 → JSON,三層剝殼後 amsl 還要在。"""
        argv = rcs.build_publish_command(LAT, LON, amsl=0.0)
        assert _payload_of(argv)["amsl"] == pytest.approx(0.0)

    def test_land_command_has_no_amsl(self):
        argv = rcs.build_publish_command(LAT, LON)
        assert "amsl" not in _payload_of(argv)

    def test_selfcheck_catches_a_dropped_amsl(self):
        """自我對帳必須抓得到「說要送卻沒送」—— 這正是原本漏掉的那一類。"""
        script = "ros2 topic pub --once -w 1 %s %s %s" % (
            rcs.TARGET_TOPIC, rcs.STRING_MSG,
            rcs.ros2_string_arg(rcs.build_target_payload(LAT, LON)))
        with pytest.raises(rcs.StotBridgeError):
            rcs._selfcheck_roundtrip(script, LAT, LON, amsl=0.0)

    def test_selfcheck_catches_an_unexpected_amsl(self):
        script = "ros2 topic pub --once -w 1 %s %s %s" % (
            rcs.TARGET_TOPIC, rcs.STRING_MSG,
            rcs.ros2_string_arg(rcs.build_target_payload(LAT, LON, amsl=0.0)))
        with pytest.raises(rcs.StotBridgeError):
            rcs._selfcheck_roundtrip(script, LAT, LON, amsl=None)

    def test_selfcheck_passes_when_they_match(self):
        for amsl in (None, 0.0, 77.5):
            argv = rcs.build_publish_command(LAT, LON, amsl=amsl)
            rcs._selfcheck_roundtrip(" ".join(argv), LAT, LON, amsl=amsl)


class TestConfirmTextTellsTheTruth:
    def test_sea_variant_states_the_pinned_datum(self):
        t = rcs.confirm_text(LAT, LON, amsl=0.0)
        assert "amsl = 0.0 m" in t
        assert '"amsl": 0.0' in t, "送出的 JSON 原文也要攤開"

    def test_land_variant_warns_about_water(self):
        """⚠️「沒有 amsl 欄位」看起來像沒事,實際上是「回頭查地形庫」。
        海上那是垃圾,所以這個情形一定要主動說。"""
        t = rcs.confirm_text(LAT, LON)
        assert "地形庫" in t
        assert "水域" in t

    def test_the_two_variants_differ(self):
        assert rcs.confirm_text(LAT, LON, amsl=0.0) != rcs.confirm_text(LAT, LON)


class TestControllerMakesTheUserChoose:
    def test_two_action_buttons_exist(self):
        assert "🌊 海上目標" in CTRL
        assert "⛰ 陸上目標" in CTRL

    def test_cancel_is_the_default_and_escape(self):
        """⚠️ 打擊是不可逆動作,預設鈕永遠是取消。"""
        assert "setDefaultButton(b_cancel)" in CTRL
        assert "setEscapeButton(b_cancel)" in CTRL

    def test_no_silent_default_datum(self):
        """不可以出現「使用者沒選就幫他挑一個」的路徑。"""
        assert re.search(r"amsl = rcs\.SEA_LEVEL_AMSL if picked is b_sea else None", CTRL)
        assert "picked is b_cancel or picked is None" in CTRL

    def test_amsl_is_threaded_to_publish(self):
        assert "self._rcs_publish(la, lo, approach, amsl=amsl)" in CTRL
        assert "amsl=amsl" in CTRL

    def test_shown_text_matches_the_button(self):
        """⚠️ 不可以「看到 A 送出 B」:主文案是海上版,陸上版放在詳情裡,
        而兩顆鈕各自送自己那一種。"""
        assert "box.setText(_body(rcs.SEA_LEVEL_AMSL))" in CTRL
        assert "box.setDetailedText" in CTRL

    def test_sea_level_constant_is_zero(self):
        assert rcs.SEA_LEVEL_AMSL == 0.0


class TestNoHardcodedAircraftCount:
    """使用者定調:數量不要寫死。這一檔 2026-08-07 之前有五處寫死「六架」,
    n=2 的航次上確認框會對著兩架飛機說「會立刻帶著六架進入 ALIGN」。"""

    SRC = (ROOT / "integrations" / "rclick_strike.py").read_text(encoding="utf-8")

    def test_no_六架_in_user_facing_strings(self):
        """⚠️ 用 ast 找「非 docstring 的字串常數」。

        行為導向的粗篩(切掉 `#` 之後看整行)會把 **docstring 裡的說明**
        也算進去 —— 那些正是解釋這個 bug 的文字,本來就該留著。
        真正要抓的是會被印給使用者看的字串字面值。
        """
        import ast
        tree = ast.parse(self.SRC)
        docs = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.ClassDef,
                                 ast.FunctionDef, ast.AsyncFunctionDef)):
                d = ast.get_docstring(node, clean=False)
                if d is not None:
                    docs.add(d)
        bad = [n.value for n in ast.walk(tree)
               if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and "六架" in n.value and n.value not in docs]
        assert not bad, "使用者可見字串仍寫死六架:%r" % (bad,)

    @pytest.mark.parametrize("n,expect", [
        (2, "2 架"), (3, "3 架"), (6, "6 架"), (12, "12 架"),
    ])
    def test_confirm_text_uses_the_real_count(self, n, expect):
        assert expect in rcs.confirm_text(LAT, LON, amsl=0.0, n_aircraft=n)

    def test_unknown_count_says_so_instead_of_guessing(self):
        """⚠️ 拿不到就講「全部機體」—— 猜一個數字比不講更糟。"""
        t = rcs.confirm_text(LAT, LON, amsl=0.0)
        assert "全部機體" in t
        assert "6 架" not in t

    def test_report_reads_n_from_tot_status(self):
        rep = rcs.StandbyReport(status={"phase": "STANDBY", "n": 2})
        assert rep.swarm_n == 2

    def test_report_returns_none_when_status_missing(self):
        assert rcs.StandbyReport().swarm_n is None
        assert rcs.StandbyReport(status={"phase": "STANDBY"}).swarm_n is None

    def test_preflight_count_advisory_uses_the_node_n(self):
        """arduplane=2 且節點 n=2 ⇒ **不該**再跳「預期 6」那條假警告。"""
        rep = rcs.StandbyReport(arduplane=2, tot_node=1, ros2_cli=1, mavros=2,
                                sub_count=1, topic_rc=0, ok=True,
                                status={"phase": "STANDBY", "n": 2})
        ids = {i.code for i in rcs.evaluate_preflight(rep, target=(LAT, LON))}
        assert "SITL_COUNT" not in ids

    def test_mismatch_still_reported(self):
        rep = rcs.StandbyReport(arduplane=2, tot_node=1, ros2_cli=1, mavros=2,
                                sub_count=1, topic_rc=0, ok=True,
                                status={"phase": "STANDBY", "n": 6})
        ids = {i.code for i in rcs.evaluate_preflight(rep, target=(LAT, LON))}
        assert "SITL_COUNT" in ids

    def test_no_advisory_when_count_is_unknowable(self):
        """兩邊都不知道就別報 —— 拿常數 6 去比會製造每一次都跳的假警告。"""
        rep = rcs.StandbyReport(arduplane=2, tot_node=1, ros2_cli=1, mavros=2,
                                sub_count=1, topic_rc=0, ok=True)
        ids = {i.code for i in rcs.evaluate_preflight(rep, target=(LAT, LON))}
        assert "SITL_COUNT" not in ids

    def test_explicit_expectation_still_wins(self):
        rep = rcs.StandbyReport(arduplane=2, tot_node=1, ros2_cli=1, mavros=2,
                                sub_count=1, topic_rc=0, ok=True,
                                status={"phase": "STANDBY", "n": 2})
        ids = {i.code for i in rcs.evaluate_preflight(
            rep, target=(LAT, LON), expect_sitl=6)}
        assert "SITL_COUNT" in ids

    def test_controller_passes_the_real_count(self):
        assert "n_aircraft=report.swarm_n" in CTRL
