# -*- coding: utf-8 -*-
"""Zarchan listing 抽取器的迴歸鎖(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
`matlab/zarchan/extract_listings.py` 把全書 98 個 listing 的原始碼從
`_source/zarchan.txt`(1,552,668 字元)切出來並修 OCR。所有移植工作都以它的
輸出為來源 ⇒ 它退步一次,後面每一份移植都會建立在壞掉的來源上,而且**不會有
任何症狀** —— 移植品照樣通過自己的基準,只是實作的不是書上那一支。

⚠️⚠️ 建這支的過程踩到四個坑,**每一個都是靜默的**,四條都在下面有鎖:

  ① 標題含 en-dash ⇒ 漏掉 3.1 / 5.1 / 28.4(98 抽成 94);
  ② 圖說夾在程式碼中間 ⇒ 遇散文即 break ⇒ 6.2 抽到 2 行、14.1 抽到 1 行,
     而兩者都「看起來有內容」;
  ③ 從尾端往回修剪 ⇒ 式子含 ¼ð Þ 不算散文 ⇒ 34.4 抽出 1159 行(後面整本書);
  ④ 用行數判過短 ⇒ 原文的碼擠在超長行裡,行數零鑑別力。

⭐ 還有一條**方法論**的鎖:重排規則加了之後 MATLAB 可解析數從 12 掉到 0。
   規則本身「看起來明顯該加」。⇒ 改重排一定要重量,不可以憑直覺。
"""
import json
import pathlib
import re
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
EXT = ROOT / "matlab" / "zarchan" / "extract_listings.py"
SRC = ROOT / "matlab" / "zarchan" / "_source" / "zarchan.txt"
OUT = ROOT / "matlab" / "zarchan" / "_listings"

pytestmark = pytest.mark.skipif(
    not SRC.exists(), reason="matlab/zarchan/_source 不在這個 checkout 裡")


@pytest.fixture(scope="module")
def mod():
    sys.path.insert(0, str(EXT.parent))
    import extract_listings as m                                # noqa: E402
    return m


@pytest.fixture(scope="module")
def items(mod):
    return mod.extract(SRC.read_text(encoding="utf-8", errors="replace"))


class TestEveryListingIsFound:
    def test_there_are_exactly_98(self, items):
        """⚠️ 98 是**數出來的**:全文的大寫 LISTING 標題數。
        變少 = 有 listing 被漏掉(靜默);變多 = 錨點抓到了不該抓的東西。"""
        assert len(items) == 98, (
            f"抽到 {len(items)} 個。⚠️ 第一版抽到 94 —— 少的四個"
            "(3.1 / 5.1 / 28.4 / 30.3)標題裡有 en-dash 或超過長度上限。")

    def test_the_en_dash_titles_are_not_lost(self, items):
        """⭐ 這四個是 ① 號坑的實例。它們的標題把 "SECOND-ORDER" 排成
        "SECOND–ORDER"(U+2013),字元類少了它就整個漏掉。"""
        got = {(i["ch"], i["no"]) for i in items}
        for k in ((3, 1), (5, 1), (28, 4), (30, 3)):
            assert k in got, f"Listing {k[0]}.{k[1]} 又漏了 —— 檢查 HEAD 的字元類"

    def test_numbers_are_unique(self, items):
        seen = [(i["ch"], i["no"]) for i in items]
        assert len(set(seen)) == len(seen), "有重複編號 ⇒ 錨點把交叉引用當標題了"


class TestNothingIsSilentlyTruncated:
    """⚠️⚠️ ② 號坑:截斷**不會報錯**,檔案看起來有內容,只是少了九成。"""

    #: 這兩個是實際被腰斬過的。6.2 曾經只剩 175 字元、14.1 只剩 397。
    ONCE_TRUNCATED = {(6, 2): 900, (14, 1): 2000}

    @pytest.mark.parametrize("key,floor", sorted(ONCE_TRUNCATED.items()))
    def test_the_known_truncations_stay_fixed(self, items, key, floor):
        it = next(i for i in items if (i["ch"], i["no"]) == key)
        assert len(it["raw"]) > floor, (
            f"Listing {key[0]}.{key[1]} 只有 {len(it['raw'])} 字元 —— 又被腰斬了。"
            "成因是圖說(Fig. N.M ...)夾在程式碼中間被判成散文。")

    def test_nothing_swallowed_the_rest_of_the_book(self, items):
        """⚠️ ③ 號坑的反向:34.4 曾經抽出 1159 行。
        listing 再長也不會有那個量級。"""
        worst = max(items, key=lambda i: len(i["raw"]))
        assert len(worst["raw"]) < 30000, (
            f"Listing {worst['ch']}.{worst['no']} 抽出 {len(worst['raw'])} 字元 —— "
            "切點失效,把後面的正文吃進來了。")

    def test_the_total_is_the_right_order_of_magnitude(self, items):
        tot = sum(len(i["raw"]) for i in items)
        assert 200_000 < tot < 400_000, (
            f"98 個 listing 共 {tot} 字元 —— 量級不對。"
            "太少 = 普遍截斷;太多 = 切點失效。")


class TestTheOcrRepairsAreTheOnesWeVerified:
    """⭐ 用 Listing 2.1 當金標:它的原文我逐字核對過。"""

    @pytest.fixture(scope="class")
    def l21(self, mod, items):
        it = next(i for i in items if (i["ch"], i["no"]) == (2, 1))
        return mod.repair(it["raw"])[0]

    @pytest.mark.parametrize("want,was", [
        ("while VC >= 0", "while VC .= 0        ( > 被 OCR 成 . )"),
        ("if RTM < 1000", "if RTM , 1000        ( < 被 OCR 成 , )"),
        ("while STEP <= 1", "while STEP ,=1"),
        ("if S >= .09999", "if S . =.09999       (`.` 與 `=` 中間還夾空格)"),
    ])
    def test_a_comparison_operator_came_back(self, l21, want, was):
        assert want in l21, f"沒有還原成 {want!r};原文是 {was}"

    def test_the_guidance_law_line_is_intact(self, l21):
        """⭐⭐ 這一行是整本書的核心。它壞了,所有移植都建在沙上。"""
        assert "XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM)" in l21
        assert "XNC=XNP*VC*XLAMD" in l21
        assert "AM1=-XNC*sin(XLAM); AM2=XNC*cos(XLAM)" in l21

    def test_the_fine_step_trigger_is_range_not_time(self, l21):
        """⚠️ 書上用的是**距離**(RTM < 1000 ft),不是時間。
        我先前的簡報寫成 `TGO < 0.3 s` —— 錯的。"""
        assert "if RTM < 1000, H=.0002;" in l21
        assert "H=.01;" in l21, "粗步是 0.01(不是 0.001)"

    def test_typographic_quotes_became_ascii(self, l21):
        assert "’" not in l21 and "‘" not in l21
        assert "title('Two-dimensional" in l21

    def test_the_relayout_inserted_separators(self, l21):
        """`end BETAOLD=BETA` 與 `if FLAG==1 STEP=2` 在 MATLAB 都是語法錯 ——
        書上是分行的,抽取時被併掉了。"""
        assert "if FLAG==1, STEP=2;" in l21, "if 條件後面要補逗號"
        assert re.search(r"end\nBETAOLD=BETA", l21), "end 後面要補換行"


class TestTheRepairsNeverGuess:
    """⚠️ 「修得有把握的、標記修不了的、絕不猜」—— 這是本檔的立場,要有鎖。"""

    def test_a_variable_to_variable_comparison_is_flagged_not_guessed(self, mod):
        """`if XLONGM.XLONGT` 從形狀判斷不出是 < 還是 >。猜一個會給出
        一個跑得動、答案相反的模擬 —— 那比語法錯嚴重得多。"""
        fixed, _, left = mod.repair("if XLONGM.XLONGT\nX=1;\nend")
        assert "XLONGM.XLONGT" in fixed, "不可以猜方向"
        assert any("無法從形狀判斷方向" in s for s in left), "但一定要標出來"

    def test_a_real_decimal_point_is_not_touched(self, mod):
        """⚠️ 把 `.` 全域換成 `>` 會毀掉每一個小數與 atan2 的逗號 ——
        那是「修好一個、弄壞五十個」。"""
        code = "H=.0002; XLAM=atan2(RTM2,RTM1); VC=-(A*B + C*D)/RTM;"
        fixed, _, _ = mod.repair(code)
        assert fixed == code, f"動到了不該動的:{fixed!r}"

    def test_prose_equations_are_reported_not_converted(self, mod):
        """正文式子的 ligature(¼ð Þþ½)**不是**程式碼,不可以就地改成碼。"""
        _, _, left = mod.repair("FðsÞ ¼ ð1 0 fðtÞest dt")
        assert any("ligature" in s for s in left)


class TestTheOutputOnDiskMatchesTheExtractor:
    """⭐ 磁碟上的 98 份是移植工作的來源。它與抽取器脫節就是最壞的情況:
    有人改了抽取器、沒重跑,而移植繼續用舊的。"""

    def test_the_index_and_the_files_agree(self):
        idx_path = OUT / "_index.json"
        if not idx_path.exists():
            pytest.skip("尚未產出;跑 python matlab/zarchan/extract_listings.py")
        idx = json.loads(idx_path.read_text(encoding="utf-8"))
        assert len(idx) == 98
        missing = [r["file"] for r in idx if not (OUT / r["file"]).exists()]
        assert not missing, f"index 記了但檔案不在:{missing}"

    def test_rerunning_is_idempotent(self):
        """跑兩次要得到一樣的東西。不然「重跑一次看看」會變成一種賭博。"""
        if not (OUT / "_index.json").exists():
            pytest.skip("尚未產出")
        before = (OUT / "listing_02_01.m").read_bytes()
        r = subprocess.run([sys.executable, str(EXT)], capture_output=True,
                           cwd=str(ROOT), text=True, encoding="utf-8",
                           errors="replace")
        assert r.returncode == 0, r.stdout + r.stderr
        assert (OUT / "listing_02_01.m").read_bytes() == before, "重跑產出不同 ⇒ 不冪等"
