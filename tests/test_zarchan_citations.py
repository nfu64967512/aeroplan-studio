# -*- coding: utf-8 -*-
r"""每一條 `zarchan.txt:NNNN` 引用都必須用 **grep 的行號慣例**。

⚠️⚠️ 這個檔在守什麼
────────────────────
`zarchan.txt` 有 15120 個 LF **和 15307 個 CR**(15120 個 CRLF + 187 個裸 CR)。
於是**同一個檔**有兩種行號:

    grep / sed / Read        只切 LF        -> 15120 行
    MATLAB readlines()       裸 CR 也切     -> 15308 行

同一句話("the miss distance results get better")在前者是 **10837** 行、
在後者是 **11024** 行,差 **+187**,而且差值隨位置累積(前面沒有裸 CR 時是 0)。

⇒ 拿 MATLAB 找行號的 agent 會寫出 `zarchan.txt:11024`,而任何用 grep 的人
   在那一行看到的是**完全不同的文字**。引用看起來仍然像個引用 —— 這正是
   最壞的那種錯:它不會報錯,只會讓下一個人查證失敗然後懷疑自己。

實際發生過:`kalmanLawOptions` 原本引用 `:11024`,`dualRateForward` 引用
`:2087`(落在一個圖片連結上),`kalmanLawAdjoint` / `dualRateAdjoint` 整批
用 readlines 索引卻標著 `zarchan_decoded.txt:NNNN`。

⚠️ 這**不是**兩個書檔對不齊的問題 —— 那個已經修好了(兩檔現在 LF/CR 完全
   相同,同一句話兩邊同一行)。這是**單一檔案的兩種讀法**。

守法:抓出移植檔裡「引用 + 附近的引號字串」成對出現的地方,驗那句話
真的在那一行(grep 慣例)。查不到就再試 readlines 慣例 —— 如果換成
readlines 就對上了,那就百分之百確定是這個病,錯誤訊息直接把正確行號給你。

⚠️⚠️ **這是正向偵測器,不是普查。** 83 個移植裡只有 **12** 個有「引用 +
書上真句子」成對出現而驗得動,其餘 71 個被 skip。
⇒ **開火時是確定的;沉默時什麼都不代表。**
不要拿「全綠」當成「所有引用都對」—— 那正是這條守衛自己差點犯的錯。

⚠️ 兩次收窄都是被誤報逼出來的,而兩次的病因都是「把不是引文的東西當引文」:
  ① 移植英文散文裡的**所有格撇號**開出假字串(`the airframe's roots are`
     -> `s roots are the airframe`)⇒ 63 個裡 45 個被判「對不上」。
     修法:片語不在書裡就沒有證據力,跳過。
  ② 移植自己的**列名**剛好也印在書上(`target maneuver magnitude` 對上
     名詞表那一行)⇒ 一個 12 條引用全對的移植被判假紅燈。
     修法:值得引用的句子比列名長 —— 門檻提到 40 字元且至少 5 個詞。
每收窄一次召回率就掉一截,而這是刻意的:誤報高的守衛會被關掉,
被關掉的守衛連它真正要抓的那一類也一起漏。
"""
import io
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "matlab" / "mgs" / "+mgs" / "+zarchan"
SRC = ROOT / "matlab" / "zarchan" / "_source" / "zarchan.txt"

#: 引用的樣子:zarchan.txt:1234 / zarchan_decoded.txt:1234 / :1234-1240
CITE = re.compile(r"zarchan(?:_decoded)?\.txt:(\d+)")

#: 引號字串,長到足以當指紋。MATLAB 用單引號,書上的句子常含撇號 -> ''。
#:
#: ⚠️⚠️ 門檻是 40 字元 **加上** 至少 5 個詞,而不是原本的 24 字元。
#:   24 太短會把移植自己的**列名**當成引文:`discreteRiccati` 有一列叫
#:   'target maneuver magnitude'(24 字元、3 個詞),而書上的名詞表剛好也有
#:   一行「nT Target maneuver magnitude 5 g」⇒ 指紋「存在於書中」的檢查通過,
#:   於是它被拿去驗一條**不是它的**引用,判了這個移植一個假紅燈 ——
#:   而那個移植的 12 條引用在 grep 慣例下全部正確。
#:
#:   判別的道理:**值得引用的句子比列名長。** 三個真的命中都在 41-66 字元、
#:   5-10 個詞,所以提高門檻不會漏掉它們(已驗)。
QUOTE = re.compile(r"'((?:[^'\n]|''){40,200})'")
MIN_WORDS = 5

#: 引用與引號之間容許的距離(字元)。同一列/同一段註解通常在這個範圍內。
NEAR = 400


def _ports():
    if not PKG.is_dir():
        return []
    return sorted(p for p in PKG.glob("*.m") if p.name != "benchSuite.m")


@pytest.fixture(scope="module")
def book():
    if not SRC.exists():
        pytest.skip("沒有 zarchan.txt")
    raw = SRC.read_bytes()
    # grep 慣例:只切 LF
    grep_lines = raw.split(b"\n")
    # readlines 慣例:CRLF、裸 CR、LF 都切
    rl_lines = re.split(rb"\r\n|\r|\n", raw)
    return grep_lines, rl_lines


def _norm(s):
    """比對前把 MATLAB 的 '' 還原成 ',並壓掉空白差異。"""
    return re.sub(r"\s+", " ", s.replace("''", "'")).strip().lower()


def _find(lines, needle, idx, span=3):
    """needle 是否出現在 lines[idx-1] 附近 +/- span 行(1-based idx)。"""
    lo, hi = max(0, idx - 1 - span), min(len(lines), idx - 1 + span + 1)
    blob = b" ".join(lines[lo:hi]).decode("utf-8", "replace")
    return needle in _norm(blob)


@pytest.mark.parametrize("path", _ports(), ids=lambda p: p.stem)
def test_every_citation_uses_the_grep_line_convention(path, book):
    grep_lines, rl_lines = book
    text = path.read_text(encoding="utf-8", errors="replace")

    # ⚠️⚠️ 指紋必須先證明它**是書上的句子**。
    #   第一版沒有這一關,結果 63 個移植裡 45 個被判成「指紋都對不上」——
    #   而那些「指紋」根本不是引文,是移植自己英文散文裡的**所有格撇號**
    #   開出來的假字串(`the airframe's roots are ...` -> `s roots are the
    #   airframe`)。同一類詞法錯誤在 test_zarchan_port_lint 已經犯過三次。
    #
    #   規則很簡單而且是決定性的:**一個在整本書裡根本不存在的片語,對行號
    #   沒有任何證據力。** 跳過它,不要算成「對不上」——把雜訊算成證據,
    #   守衛就會在自己的誤報裡淹死,然後被關掉。
    book_blob = _norm(b"\n".join(grep_lines).decode("utf-8", "replace"))
    pairs = []
    for m in CITE.finditer(text):
        n = int(m.group(1))
        window = text[max(0, m.start() - NEAR):m.end() + NEAR]
        for q in QUOTE.finditer(window):
            frag = _norm(q.group(1))
            if len(frag.split()) < MIN_WORDS or re.search(r"[{}()\[\];=]", frag[:40]):
                continue
            if frag not in book_blob:          # <- 不是引文,沒有證據力
                continue
            pairs.append((n, frag))
    if not pairs:
        pytest.skip("這個移植沒有『引用 + 書上真句子』成對出現的地方")

    wrong = []
    for n, frag in pairs:
        if n > len(grep_lines):
            wrong.append((n, frag, "超出 grep 行數 %d" % len(grep_lines)))
            continue
        if _find(grep_lines, frag, n):
            continue                                   # 對得上,收工
        # 對不上 —— 換 readlines 慣例試試看
        if n <= len(rl_lines) and _find(rl_lines, frag, n):
            # 反算它在 grep 慣例下應該是第幾行
            pos = sum(len(x) + 1 for x in rl_lines[:n - 1])
            real = SRC.read_bytes()[:pos].count(b"\n") + 1
            wrong.append((n, frag,
                          "這是 **readlines** 索引;grep 慣例應為 :%d" % real))
            continue
        # 這句話**在書裡**,卻不在被引用的那一行附近,兩種慣例都不是。
        # ⚠️ 但這一類**不能硬斷**,因為有兩種合理的無辜解釋:
        #   ① 引用指向**段落起點**而句子在幾行之後 —— 那是正常的引用習慣。
        #      實測 kalmanLawOptions 差 7 行、dualRateForward 差 22 行,
        #      兩個都是這種。
        #   ② 片語太通用 —— optimalGainsWeave 的「target maneuver amplitude」
        #      在書裡出現 3 次,拿它當指紋等於擲骰子。
        # ⇒ 只在**決定性**的時候才算數:片語在全書唯一,而且差距大到不可能
        #   是指向段落起點(> 50 行)。寧可漏,不可誤報 —— 誤報高的守衛會被
        #   關掉,然後連 A 類那種確定的也一起漏掉。
        where = [i + 1 for i, ln in enumerate(grep_lines)
                 if frag in _norm(ln.decode("utf-8", "replace"))]
        if len(where) == 1 and abs(where[0] - n) > 50:
            wrong.append((n, frag,
                          "這句話在書裡**唯一一處**,是第 %d 行,離 :%d 有 %d 行 "
                          "⇒ 不可能是指向段落起點" % (where[0], n, abs(where[0] - n))))

    assert not wrong, (
        "⚠️⚠️ %s 有 %d 條引用用了 MATLAB readlines 的行號,而不是 grep 的:\n%s\n"
        "    同一個檔有兩種行號(LF 15120 vs LF+裸CR 15308,最大差 +187)。\n"
        "    用 grep 的人在那一行會看到完全不同的文字,而引用看起來仍然\n"
        "    像個引用 —— 不會報錯,只會讓下一個查證的人懷疑自己。"
        % (path.name, len(wrong),
           "\n".join("      :%d  %r\n        -> %s" % (n, f[:70], w)
                     for n, f, w in wrong[:8])))


def test_the_two_line_conventions_really_do_diverge(book):
    r"""⭐ 先證明這條守衛守的是**真的**東西,免得它是在防一個不存在的病。

    ⚠️ 這條刻意寫成「兩種慣例必須不同」。哪天有人把 zarchan.txt 正規化成
       純 LF,這條會紅 —— 那是對的:到那天為止,上面那條守衛都是必要的,
       而那天之後它就可以退休了。
    """
    grep_lines, rl_lines = book
    assert len(rl_lines) > len(grep_lines), (
        "⚠️ 兩種行號慣例現在一致了(%d == %d)。若 zarchan.txt 被正規化成純 LF,"
        "上面那條引用守衛就不再需要 —— 但請確認那是**刻意**的,"
        "因為既有移植的引用是照舊慣例寫的。" % (len(rl_lines), len(grep_lines)))
    probe = b"the miss distance results get better"
    raw = SRC.read_bytes()
    i = raw.find(probe)
    assert i > 0
    g = raw.count(b"\n", 0, i) + 1
    r = len(re.split(rb"\r\n|\r|\n", raw[:i]))
    assert r - g == 187, (
        "⚠️ 兩種慣例的差變成 %+d(記錄時是 +187)。裸 CR 的數量變了,"
        "既有引用的可信度要重新評估。" % (r - g))
