# -*- coding: utf-8 -*-
"""航次產物必須**自足** —— 缺一個檔就會讓人去讀錯的數字。

═══ 2026-08-11 的教訓 ═══

`rep_interactive.sh` 收了 `tot_<TAG>.csv` 與 `tot_standby.log`,**漏收 geom**。
後果不是「少一個檔」:

    geom 缺席 ⇒ stot_score.py 直接 return 2(找不到目標座標)
             ⇒ 只剩結案訊息的「最近水平距」可讀
             ⇒ 而那個值是**鎖存當下凍結**的,訊息自己標明它是
               「真最近點的**上界**,誤差 ≤ 一個取樣步」
             ⇒ 末端 ~120 m/s ⇒ 一個取樣步就十幾公尺

RPJ 那一批因此被讀成「誤差 26–32 m,系統性偏差」,並據此做了錯的優先序判斷;
補上 geom 用內插重量之後,真值是 **0.03–0.86 m**,門檻③ 其實早就通過。

⇒ 產物完整性不是整潔問題,是**結論正確性**問題。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
W = ROOT / "scripts" / "wsl"
REP = (W / "rep_interactive.sh").read_text(encoding="utf-8")
TOT = (W / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")

# geom 的消費者:任何一支缺了輸入都會靜默失效
CONSUMERS = [
    "stot_score.py",
    "impact_feasibility.py",
    "spread_growth.py",
    "_tot_plot.py",
    "_dive_plot.py",
]


def _code_lines(src):
    """去掉註解,避免測試讀到說明文字而誤判通過(本專案犯過三次)。"""
    return "\n".join(ln.split("#", 1)[0] for ln in src.splitlines())


class TestGeomIsProducedAndCollected:
    def test_tot_node_writes_geom(self):
        code = _code_lines(TOT)
        assert "_write_geom_csv" in code, "tot_node 不再產生 geom?"
        assert '"_geom.csv"' in code or "_geom.csv" in code

    def test_geom_has_the_target_row(self):
        """消費端一律找 kind == 'target';欄名或列名改了就全部失效。"""
        code = _code_lines(TOT)
        assert re.search(r'\[\s*"kind"\s*,\s*"lat"\s*,\s*"lon"\s*\]', code), \
            "geom 的表頭不是 kind,lat,lon"
        assert re.search(r'\[\s*"target"\s*,', code), "geom 沒有寫 target 那一列"

    def test_rep_interactive_collects_geom(self):
        code = _code_lines(REP)
        assert "tot_standby_geom.csv" in code, (
            "rep_interactive.sh 沒有收 geom —— 評分工具會全部跑不起來,"
            "只剩結案訊息的鎖存上界可讀(RPJ 就是這樣被讀成 26–32 m)"
        )
        assert re.search(r'_geom\.csv"?\s*$|_geom\.csv"\s', code, re.M), \
            "收檔的目標檔名不含 _geom.csv"

    def test_collected_geom_matches_the_name_scorer_expects(self):
        """`stot_score.py` 預設找 `<csv 去掉 .csv>_geom.csv`,兩邊必須對得上。"""
        scorer = (W / "stot_score.py").read_text(encoding="utf-8")
        assert 'replace(".csv", "_geom.csv")' in scorer, \
            "stot_score.py 推導 geom 檔名的方式改了,收檔命名要跟著改"
        code = _code_lines(REP)
        assert 'tot_${TAG}.csv' in code and 'tot_${TAG}_geom.csv' in code, (
            "收檔命名必須是 tot_<TAG>.csv 與 tot_<TAG>_geom.csv,"
            "否則 stot_score.py 的預設推導找不到"
        )


class TestScoringHappensInBand:
    """⚠️ 評分要**當場**做:留給事後就會有人去讀鎖存值(我就讀錯了)。"""

    def test_rep_interactive_runs_the_scorer(self):
        code = _code_lines(REP)
        assert "stot_score.py" in code, (
            "rep_interactive.sh 沒有跑 stot_score.py —— "
            "摘要裡只有鎖存上界,判讀一定會偏"
        )

    @pytest.mark.parametrize("consumer", CONSUMERS)
    def test_consumer_exists(self, consumer):
        """清單是手維護的;檔案改名要讓這裡紅,而不是靜默少驗一支。"""
        assert (W / consumer).exists(), f"{consumer} 不見了 —— 更新 CONSUMERS"
