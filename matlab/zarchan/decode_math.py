# -*- coding: utf-8 -*-
r"""把 zarchan.txt 的數學字體錯映還原,產生 zarchan_decoded.txt。

⚠️⚠️ 為什麼需要這個檔
─────────────────────
書的 PDF 在**數學區**用了另一套字體,抽取時整區被錯映成五個字元。全書 6247 次:

    ¼ → =   (2196)      ð → (   (1454)      Þ → )   (1305)
    þ → +   (1157)      ½ → [   ( 135)

後果是:**方程式與行內數學裡的數值,naive grep 一律找不到**。
`b ¼ 0:8` 這一行,你 grep `b = 0.8` 得到零筆,然後照「散文有、圖上沒有」
那條規矩判成「這個值只存在於圖上 ⇒ 誠實拒評」——但它其實印在書上。

⚠️ 已查證:這**沒有**推翻任何一條既有的 SKIP(第 24 章 D/E/F 的幾何矛盾、
   第 20 章 20.17-20.19 的無數值、第 22 章 Fig 22.6 的頻帶,三條都仍然成立)。
   它的價值在往後:可 grep 的 `<變數> = <數值>` 從 806 增為 1088(+35%)。

⚠️⚠️ 小數點那條規則**必須收窄**
─────────────────────────────
數學區的小數點也被寫成冒號(`32:2` 就是 `32.2`)。但書裡同樣有大量
**MATLAB 冒號範圍**:

    for N=1:20        for RT1IC=500:500:40000        for XLATTDEG=-60:2.5:60

無差別的 `(?<=\d):(?=\d)` → `.` 會把 `for N=1:20` 變成 `for N=1.20`,
**默默毀掉書裡每一個迴圈範圍**,而這個檔正是之後每一波要 grep 的東西。

判別法就在資料裡:數學區周圍一定有那五個錯映字元,程式碼區沒有
(碼是完好抽出來的)。所以只在**視窗內出現錯映字元**時才換小數點。

原檔一個位元都不動。要重建就重跑這支。
"""
import io
import os
import re
import sys

MAP = {"¼": "=", "ð": "(", "Þ": ")", "þ": "+", "½": "["}
MARK = "".join(MAP)
WINDOW = 120          # 判斷「這裡是數學區嗎」的左右視窗,單位字元


def code_spans(text):
    """回傳 [(start, end), ...] —— 每個 LISTING 的**程式碼**在 text 裡的位移。

    ⚠️ 借 extract_listings 的同一組錨點與同一條結束規則,而不是另寫一套。
       兩套規則一定會漂,而漂掉的那天沒有人會發現。
    """
    import extract_listings as X
    spans = []
    # ⚠️ 用 HEAD_ANY 不是 HEAD:附錄的 8 個 listing(A.1~A.8)也是碼。
    #    漏掉它們的後果是實測到的 —— `for I=2:160` 被解碼成 `for I=2.160`、
    #    `for DISTKM=100:100:20000` 變 `for DISTKM=100.100:20000`。
    heads = list(X.HEAD_ANY.finditer(text))
    for k, m in enumerate(heads):
        stop = heads[k + 1].start() if k + 1 < len(heads) else len(text)
        pos, run, end = m.end(), 0, m.end()
        for line in text[m.end():stop].split("\n"):
            nxt = pos + len(line) + 1
            if any(pt.match(line) for pt in X.NOISE):
                pos = nxt
                continue
            if X._prose(line):
                run += 1
                if run >= X.PROSE_RUN:
                    break
            else:
                run = 0
                end = nxt
            pos = nxt
        if end > m.end():
            spans.append((m.end(), end))
    return spans


def decode(src):
    """回傳 (解碼後文字, 統計 dict)。"""
    stats = {ch: src.count(ch) for ch in MAP}
    spans = code_spans(src)
    stats["code_spans"] = len(spans)

    # ⚠️⚠️ 前兩版都在猜「這個冒號是範圍還是小數點」,兩個方向都錯:
    #    「附近有錯映字元才換」漏掉 7 個數學區(含書上印出來的雷達罩穩定
    #    不等式 `0:79 , ... , 2:07`);反過來「像碼就保留」把 172 個
    #    `= 0:5` 當成 `=num:` 範圍留著,那是 `= 0.5`。
    #    正解是不要猜:**碼區就是抽取器劃的 LISTING 區**,借它同一組錨點。
    #    碼區一個位元都不動(agent 本來也不從這裡移植,是從抽好的 .m 檔),
    #    區外無條件解碼。
    out, hit, kept, last = [], 0, 0, 0
    for a, b in spans + [(len(src), len(src))]:
        seg = src[last:a]
        seg = seg.translate(str.maketrans(MAP))
        seg, n = re.subn(r"(?<=\d):(?=\d)", ".", seg)
        hit += n
        out.append(seg)
        kept += src.count(":", a, b)
        out.append(src[a:b])
        last = b
    stats["decimal_swapped"] = hit
    stats["colons_untouched_in_code"] = kept
    return "".join(out), stats


def reapply_endings(processed, raw_untranslated):
    r"""把原檔的行尾**原封不動**貼回處理後的文字。

    ⚠️⚠️ 為什麼需要這一步(這是實測抓到的,不是預防性的)
    ────────────────────────────────────────────────
    `zarchan.txt` 是**混合行尾**:15120 個 CRLF **加 187 個裸 CR**。

    處理階段用 universal newlines 讀(CR 與 CRLF 都變成 LF),所以內部一切
    對齊得完美。但寫出去若統一成 LF,那 187 個裸 CR 就變成了**真的換行**:

        grep -n zarchan.txt          -> 15120 行(裸 CR 不算換行)
        grep -n zarchan_decoded.txt  -> 15307 行

    ⇒ 行號在兩個檔之間**逐漸錯開**,而且偏移不是常數(第 26.1 章 +179、
      第 26.6 章 +187,因為那時 187 個 CR 都在後面了)。

    ⚠️ 而 test_zarchan_decode 的「位移一致」那條**看不到這件事** —— 它用
       Python 文字模式讀兩邊,universal newlines 把兩邊都正規化成 LF,
       於是量到完全一致。**測試量的是正規化後的世界,agent 活在位元組的
       世界。** 這個落差是移植的審核 agent 報出來的,不是測試抓到的。

    移植的 source 欄大量引用 `zarchan.txt:1234`,而 agent 是拿 grep 找的。
    ⇒ 行尾必須一比一還原,不能只在文件裡加一句警告。
    """
    ends = re.findall(r"\r\n|\r|\n", raw_untranslated)
    lines = processed.split("\n")
    out = []
    for i, ln in enumerate(lines):
        out.append(ln)
        if i < len(ends):
            out.append(ends[i])
    return "".join(out)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src_p = os.path.join(here, "_source", "zarchan.txt")
    dst_p = os.path.join(here, "_source", "zarchan_decoded.txt")
    src = io.open(src_p, encoding="utf-8", errors="replace").read()
    raw = io.open(src_p, encoding="utf-8", errors="replace", newline="").read()
    out, st = decode(src)
    out = reapply_endings(out, raw)
    st["bytes_match_source"] = (len(out) == len(raw))
    io.open(dst_p, "w", encoding="utf-8", newline="").write(out)
    for k, v in st.items():
        print("  %-18s %6d" % (k, v))
    print("  -> %s" % dst_p)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
