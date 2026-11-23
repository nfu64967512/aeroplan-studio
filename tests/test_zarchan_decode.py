# -*- coding: utf-8 -*-
"""鎖住 zarchan.txt 的數學字體解碼。

⚠️⚠️ 這個檔在守什麼
────────────────────
書的 PDF 在**數學區**用了另一套字體,抽取時整區錯映成五個字元(全書 6247 次):

    ¼ → =    ð → (    Þ → )    þ → +    ½ → [

後果:**方程式與行內數學裡的數值,naive grep 一律找不到**。書上印 `b ¼ 0:8`,
你 grep `b = 0.8` 得到零筆,然後照「散文有、圖上沒有」那條規矩判成
「只存在於圖上 ⇒ 誠實拒評」—— 但它明明印在書上。

⚠️ 已查證:解碼**沒有**推翻任何一條既有的 SKIP(第 24 章 D/E/F 的幾何矛盾、
   第 20 章 20.17-20.19 的無數值、第 22 章 Fig 22.6 的頻帶,三條都仍成立)。
   價值在往後:可 grep 的 `<變數> = <數值>` 從 806 增為 1088(+35%)。

⚠️⚠️ 為什麼不是「聰明一點的規則」
──────────────────────────────
小數點在數學區也被寫成冒號(`32:2` 是 `32.2`),但書裡同時有大量 MATLAB
冒號範圍(`for N=1:20`)。我試過兩個方向的啟發式,**兩個都錯**:

  ① 「附近有錯映字元才換小數點」→ 漏掉 7 個數學區,其中一個是書上印出來的
     雷達罩穩定不等式 `0:79 , N 0VcRTa VMT , 2:07`。漏掉 = 把可評分的值
     判成不存在,正是這整件事要修的那個病。
  ② 反過來「長得像碼就保留」→ 把 172 個 `= 0:5`(其實是 `= 0.5`)當成
     `=num:` 範圍留著。

正解是**不要猜**:碼區就是 extract_listings 劃的 LISTING 區,借它同一組錨點。
碼區一個位元都不動,區外無條件解碼。

⚠️ 這裡的測試要能**咬**。改壞規則(換成任一種啟發式、或整個關掉)必須讓
   MUST_DECODE / MUST_KEEP 轉紅,而不是只讓計數對不上。
"""
import io
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "matlab" / "zarchan" / "_source" / "zarchan.txt"
DEC = ROOT / "matlab" / "zarchan" / "_source" / "zarchan_decoded.txt"
sys.path.insert(0, str(ROOT / "matlab" / "zarchan"))

MARK = "¼ðÞþ½"

#: 必須**保留**冒號 —— 全是 MATLAB 冒號範圍,寫在 listing 的碼裡。
#: 把它們換成小數點就是把 `for N=1:20` 寫成 `for N=1.20`,而解碼版正是
#: 之後每一波要 grep 的東西 ⇒ 靜默污染,下游會照抄。
MUST_KEEP = [
    ("MATLAB 迴圈範圍", "for N=1:20"),
    ("三段式範圍", "for I=1:1:ICOUNT"),
    ("步進範圍", "RT1IC=500:500:40000"),
    ("負起點範圍", "-100:5:200"),
]

#: 必須**解碼** —— 全是散文/數學區裡可評分的值。
#: 每一條都對應一個真實用途:漏掉就會有人把它判成「圖上才有」。
MUST_DECODE = [
    ("雷達罩穩定不等式(啟發式①漏掉的那個)", "0.79"),
    ("脫靶公式表的指數", "T 1.5F0.5"),
    ("衰退記憶因子(第 7 章標稱值)", "b = 0.8"),
    ("重力加速度", "32.2"),
    ("第 24 章的幾何(D/E/F 拒評的依據)", "RT1IC = 30000"),
    ("第 24 章的接近速度", "4000 ft/s"),
]


@pytest.fixture(scope="module")
def texts():
    if not SRC.exists():
        pytest.skip("沒有 zarchan.txt")
    if not DEC.exists():
        pytest.fail("⚠️ 缺 zarchan_decoded.txt —— 跑 matlab/zarchan/decode_math.py 重建")
    return (io.open(SRC, encoding="utf-8", errors="replace").read(),
            io.open(DEC, encoding="utf-8", errors="replace").read())


def test_decoded_file_is_in_sync_with_the_source(texts):
    """⚠️ 解碼版是**衍生檔**。來源或規則變了而沒重跑,下游就在讀舊答案。"""
    import decode_math
    src, dec = texts
    fresh, _ = decode_math.decode(src)
    assert fresh == dec, (
        "⚠️ zarchan_decoded.txt 與現在的規則對不上 ——\n"
        "    跑 `python matlab/zarchan/decode_math.py` 重建,\n"
        "    然後看 diff 確認變的是你想改的那部分。")


def test_character_offsets_are_preserved(texts):
    """⭐ 解碼是**一對一**替換,字元數不變。

    ⚠️ 這條**不足以**保證行號互通 —— 見下一條。它只保證沒有插入或刪除字元。
    """
    src, dec = texts
    assert len(src) == len(dec), (
        "⚠️ 解碼改了字元數(%d -> %d)⇒ 一對一替換的性質已經破了。"
        % (len(src), len(dec)))


def test_line_numbers_agree_at_the_BYTE_level(texts):
    r"""⭐⭐ 移植的 source 欄引用 `zarchan.txt:1234`,而 agent 是拿 grep 找的。

    ⚠️⚠️ 這條原本寫成「文字模式讀兩邊,長度與換行數相同」,而它**通過了** ——
       然後移植的審核 agent 回報行號對不上,15120 vs 15307。兩邊都沒說謊:

         zarchan.txt 是**混合行尾** —— 15120 個 CRLF **加 187 個裸 CR**。
         Python 文字模式的 universal newlines 把 CR 與 CRLF 都正規化成 LF,
         於是兩邊看起來完全一致。但 grep / wc 只認 LF:
             原檔     15120 行(裸 CR 不算換行)
             解碼版   15307 行(裸 CR 被寫成了真換行)
         而偏移**不是常數** —— 隨那 187 個 CR 累積(26.1 差 +179、26.6 差 +187)。

    ⇒ **舊測試量的是正規化後的世界,而 agent 活在位元組的世界。** 一條在錯誤的
      抽象層上量的守衛,會穩定地通過,而它要防的事照樣發生。

    ⇒ 所以這條刻意用 **bytes** 讀,並且比對三種行尾各自的數量。
    """
    p_src, p_dec = SRC.read_bytes(), DEC.read_bytes()
    for name, pat in (("LF", b"\n"), ("CRLF", b"\r\n")):
        assert p_src.count(pat) == p_dec.count(pat), (
            "⚠️ %s 的數量不一致(%d vs %d)⇒ grep 的行號會逐漸錯開。\n"
            "    解碼必須把原檔的行尾**原封不動**貼回去,不能統一成 LF。"
            % (name, p_src.count(pat), p_dec.count(pat)))
    bare_src = p_src.count(b"\r") - p_src.count(b"\r\n")
    bare_dec = p_dec.count(b"\r") - p_dec.count(b"\r\n")
    assert bare_src == bare_dec, (
        "⚠️ 裸 CR 的數量不一致(%d vs %d)—— 這正是當初 187 行偏移的來源。"
        % (bare_src, bare_dec))

    # 真正要保護的東西:同一句話在兩邊的 grep 行號相同。
    for probe in (b"LISTING 26.1", b"LISTING 26.6", b"LISTING 34.4",
                  b"LISTING A.8", b"deteriorates significantly"):
        i, j = p_src.find(probe), p_dec.find(probe)
        assert i >= 0 and j >= 0, "⚠️ 探針 %r 在某一邊找不到" % probe
        la, lb = p_src.count(b"\n", 0, i) + 1, p_dec.count(b"\n", 0, j) + 1
        assert la == lb, (
            "⚠️ %r 在原檔是第 %d 行、解碼版是第 %d 行(差 %+d)——\n"
            "    移植引用的行號在另一邊會落到別的文字上,而它看起來仍像個行號。"
            % (probe, la, lb, lb - la))


@pytest.mark.parametrize("name,needle", MUST_KEEP, ids=[n for n, _ in MUST_KEEP])
def test_colon_ranges_inside_listing_code_survive(texts, name, needle):
    """⚠️ 碼區的冒號是**範圍運算子**,不是小數點。"""
    _, dec = texts
    assert needle in dec, (
        "⚠️ 碼區的 %s(%r)在解碼版裡不見了 ——\n"
        "    冒號被當成小數點換掉了。`for N=1:20` 變 `for N=1.20`,\n"
        "    而下游 agent 是拿這個檔 grep 的,會照抄。" % (name, needle))


@pytest.mark.parametrize("name,needle", MUST_DECODE, ids=[n for n, _ in MUST_DECODE])
def test_math_region_values_become_greppable(texts, name, needle):
    """⭐ 這才是這整個檔存在的理由:讓數學區的值 grep 得到。"""
    _, dec = texts
    assert needle in dec, (
        "⚠️ %s(%r)在解碼版裡 grep 不到 ——\n"
        "    這正是要修的病:值印在書上,但寫成 `¼ 0:8`,\n"
        "    於是被判成「只有圖上有 ⇒ 誠實拒評」。" % (name, needle))


def test_no_mis_mapped_character_survives_outside_listing_code(texts):
    """⚠️ 碼區外一個錯映字元都不該剩 —— 剩下的就是 grep 不到的值。"""
    import decode_math
    src, dec = texts
    spans = decode_math.code_spans(src)
    stray = []
    for m in re.finditer("[%s]" % MARK, dec):
        i = m.start()
        if not any(a <= i < b for a, b in spans):
            stray.append(" ".join(dec[max(0, i - 50):i + 50].split()))
    assert not stray, (
        "⚠️ 碼區外還有 %d 個錯映字元:\n    %s"
        % (len(stray), "\n    ".join(stray[:6])))


def test_every_colon_range_in_the_book_survives_byte_for_byte(texts):
    """⭐⭐ 碼區界線正確與否的**值層級**判準。

    ⚠️ 這條原本寫成源碼層的鎖(檢查 decode_math.py 裡有沒有 `import
       extract_listings`)。突變實測:把那行改成
       `import extract_listings as X  # noqa` 就完全免疫 —— 0 紅。
       這個 repo 早就付過這個學費:源碼層鎖驗「那一行長什麼樣」,
       改寫法就繞過,補鎖是軍備競賽。

    改成綁**後果**:界線只要漂,就一定有 MATLAB 冒號範圍掉到碼區外被解碼。
    寫這條的當下它就抓到兩個真的 —— `for I=2:160` 和
    `for DISTKM=100:100:20000`,兩個都在**附錄**,而抽取器的錨點寫死了
    數字章節,附錄 8 個 listing 從來沒進過視野。
    """
    src, dec = texts
    pat = re.compile(r"\b(for|while)\s+\w+\s*=\s*-?[\d.]+\s*:")
    hits = list(pat.finditer(src))
    assert len(hits) > 50, "⚠️ 只找到 %d 個冒號範圍,錨點八成失效了" % len(hits)
    bad = [(m.group(0), src.count("\n", 0, m.start()) + 1)
           for m in hits if dec[m.start():m.end()] != m.group(0)]
    assert not bad, (
        "⚠️ %d 個 MATLAB 冒號範圍在解碼版上被改掉了 —— 它們掉在碼區外:\n    %s\n"
        "    冒號被當成小數點 ⇒ `for N=1:20` 變 `for N=1.20`,\n"
        "    而下游 agent 是拿這個檔 grep 的,會照抄。"
        % (len(bad), "\n    ".join("%r 行 %d" % b for b in bad[:8])))


def test_appendix_listings_are_protected_too(texts):
    """⚠️ 書裡是 **106** 個 listing,不是 98 —— 附錄還有 A.1~A.8。

    「98」是有編號章節的數量。附錄不是邊角料:A.1「ADJOINT SIMULATION OF
    OPTIMAL GUIDANCE SYSTEM」正是「tgo 尺度誤差 0.9 就讓 optimal guidance
    輸給普通 PN」那個結論的出處。

    ⚠️ 這條只保證附錄的**碼不被解碼**;附錄尚未納入抽取(改了會重寫全部 98
       個檔)。要不要移植附錄是另一個決定。
    """
    import decode_math
    src, _ = texts
    assert len(decode_math.code_spans(src)) == 106, (
        "⚠️ 碼區數不是 106 —— 附錄的 8 個 listing 沒被納入保護,\n"
        "    它們的 for/while 冒號範圍會被當成小數點解碼掉。")


def test_the_decode_actually_moves_the_needle(texts):
    """⭐ 量它到底有沒有用:可 grep 的 `<變數> = <數值>` 多了多少。

    ⚠️ 這條不是門檻,是**帳**。數字掉下來代表規則被改窄了,該去看為什麼。
    """
    src, dec = texts
    pat = re.compile(r"\b([A-Za-z][A-Za-z0-9_]{0,9})\s*=\s*(-?\d+(?:\.\d+)?)")
    before, after = len(set(pat.findall(src))), len(set(pat.findall(dec)))
    assert after > before, "⚠️ 解碼後可 grep 的等號數值沒有變多,規則八成沒生效"
    assert after - before >= 250, (
        "⚠️ 解碼只多暴露 %d 個等號數值(當初量到 +282)——\n"
        "    規則被改窄了,去看是哪一段不再解碼。" % (after - before))
