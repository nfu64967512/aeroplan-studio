#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Zarchan 全書的 98 個 listing 從抽取文字裡切出來,並修復 OCR 損壞。

用法:
    python matlab/zarchan/extract_listings.py            # 寫檔 + 報告
    python matlab/zarchan/extract_listings.py --dry      # 只報告,不寫檔

輸出:
    matlab/zarchan/_listings/listing_NN_MM.m     修復後的原始碼(可讀、未必可跑)
    matlab/zarchan/_listings/_index.json         每個 listing 的統計與未解決項

═══════════════════════════════════════════════════════════════════════════
 為什麼要有這一支
═══════════════════════════════════════════════════════════════════════════
`_source/zarchan.txt` 是整本書的抽取文字(1,552,668 字元)。2026-08-23 之前
我們以為裡面只有敘述、程式碼是圖片,所以移植 = **從散文重建**。實際 grep 之後
發現 **98 個 listing 的原始碼全部都在**(`XNC=XNP` 44 筆、`T=T+H` 74 筆、
`clc` 102 筆)⇒ 移植是**轉錄**,不是重建,難度差一個數量級。

⚠️ 但文字有三種損壞,而且**只有第一種**會自己現形:

  ① 比較運算子被吃掉 —— `<` 變 `,`、`>` 變 `.`
     `while VC .= 0` / `if RTM , 1000` / `while STEP ,=1`
     ⇒ 貼進 MATLAB 會**語法錯誤**,會被發現。
  ② 頁首頁尾與圖片連結**插進程式碼中間**
     「18 TACTICAL AND STRATEGIC MISSILE GUIDANCE」
     「https://lh3.googleusercontent.com/notebooklm/...」+ 一行 UUID
     ⇒ 也會語法錯誤。
  ③ ⚠️⚠️ **正文數學式的 ligature 錯映**(`¼`→`=`、`ð`→`(`、`Þ`→`)`、
     `þ`→`+`、`½`→`[`)。這一種**不碰程式碼**(程式碼區塊是純 ASCII),
     但如果有人拿正文的式子去實作,它會靜靜給出錯的公式。
     本檔只處理程式碼區塊,正文的修復留給讀式子的人自己注意。

⇒ 本檔的立場:**修得有把握的,標記修不了的,絕不猜。**
   每一個 listing 都附一份未解決清單;清單非空就是「這一份還不能直接用」。

⚠️ 產出的 .m **不保證可跑**。它是「書上那一頁的忠實轉錄」,
   不是移植品。移植品在 `matlab/mgs/+mgs/+zarchan/`,那裡才有基準驗證。
"""
import argparse
import json
import pathlib
import re
import sys

if sys.platform == "win32":
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass

ROOT = pathlib.Path(__file__).resolve().parents[2]
SRC = ROOT / "matlab" / "zarchan" / "_source" / "zarchan.txt"
OUT = ROOT / "matlab" / "zarchan" / "_listings"

#: listing 的程式碼區塊以**全大寫**標題起頭。
#: ⚠️ 小寫的「Listing 2.1」在正文裡出現數百次(交叉引用),不可以拿來當錨。
#: ⚠️ 字元類**必須含 en-dash**(–, U+2013):書把 "SECOND-ORDER" 排成
#:    "SECOND–ORDER"、"MISSILE–TARGET" 也是。少了它會漏掉 3.1 / 5.1 / 28.4。
#: ⚠️ 長度上限 200 而不是 140:30.3 的標題更長
#:    ("...OPTIMAL GUIDANCE LAW, UNCORRELATED GLINT NOISE AND UNIFORMLY...")。
HEAD = re.compile(
    r"^LISTING\s+(\d{1,2})\.(\d{1,2})\s+([A-Z0-9][A-Z0-9 ,\-–/&'()\.]{8,200})\s*$",
    re.M)

#: ⚠️⚠️ 書裡還有 **8 個附錄 listing**(LISTING A.1 ~ A.8),上面那條錨點
#: 寫死了數字章節,所以它們**從來沒被抽出來過**。「98 個」是有編號章節的
#: 數量,全書其實是 106 個。
#:
#: 附錄不是邊角料:A.1「ADJOINT SIMULATION OF OPTIMAL GUIDANCE SYSTEM」正是
#: 「tgo 尺度誤差 0.9 就讓 optimal guidance 輸給普通 PN」那個結論的出處。
#:
#: ⚠️ `extract()` **仍然只跑 HEAD**,附錄尚未納入抽取 —— 改了會重寫全部 98
#:    個檔,而移植正在讀它們。這裡先把錨點備好,給 decode_math 劃碼區用:
#:    附錄的碼也是碼,不保護就會被當成數學區,`for DISTKM=100:100:20000`
#:    會被解碼成 `for DISTKM=100.100:20000`。
HEAD_ANY = re.compile(
    r"^LISTING\s+([A-Z]|\d{1,2})\.(\d{1,2})\s+"
    r"([A-Z0-9][A-Z0-9 ,\-–/&'()\.]{8,200})\s*$",
    re.M)

#: 插進程式碼中間的雜訊。⚠️ 順序有意義:URL 要先於 UUID(URL 裡也有 UUID 形狀)。
NOISE = (
    re.compile(r"^\s*https?://\S+\s*$"),                       # 圖片連結
    re.compile(r"^\s*[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-"
               r"[0-9a-f]{4}-[0-9a-f]{12}\s*$"),                # 圖片 UUID
    re.compile(r"^\s*\d{1,3}\s+[A-Z][A-Z \-]{10,}\s*$"),        # 頁首「18 TACTICAL AND ...」
    re.compile(r"^\s*[A-Z][A-Z \-]{10,}\s+\d{1,3}\s*$"),        # 頁尾「... GUIDANCE 19」
    # ⚠️⚠️ 圖說**夾在程式碼中間**(排版把圖插進 listing 跨的兩頁之間)。
    #    第一版把它判成散文就 break,於是 6.2 只抽到 2 行、14.1 只抽到 1 行 ——
    #    而截斷是**靜默**的:檔案看起來有內容,只是少了九成。
    re.compile(r"^\s*Fig\.?\s*\d+\.\d+\b.*$"),
    re.compile(r"^\s*$"),
)

#: ⚠️ 只在**比較語境**裡把 `,` `.` 還原成 `<` `>`。
#:    全域取代會毀掉 `atan2(RTM2,RTM1)` 的逗號與所有小數點 ——
#:    那是「修好一個、弄壞五十個」的形狀。
#: ⚠️ `,` `.` 與 `=` 之間**可能有空白**:實測 Listing 2.1 抽出來是
#:    `if S . =.09999`(要的是 `if S >= .09999`)。第一版的樣式不吃空白,
#:    於是那一行悄悄留著一個語法錯 —— 而它長得像個小數點,不顯眼。
CMP = (
    (re.compile(r"(?<=[\w\)])\s*,\s*=\s*(?=[\w\-.])"), " <= "),
    (re.compile(r"(?<=[\w\)])\s*\.\s*=\s*(?=[\w\-.])"), " >= "),
    # `if X , 1000` / `while X , 1000`:關鍵字 + 變數 + 分隔 + 數字
    (re.compile(r"\b(if|while|elseif)\s+([A-Za-z]\w*)\s*,\s*(?=[\d.\-])"), r"\1 \2 < "),
    (re.compile(r"\b(if|while|elseif)\s+([A-Za-z]\w*)\s*\.\s*(?=[\d\-])"), r"\1 \2 > "),
)

#: 印刷用引號 → ASCII。MATLAB 的字串與轉置都用 '。
QUOTES = ((("’", "‘", "ʼ"), "'"), (("“", "”"), '"'))

#: ⭐⭐ 版面重排。書上的 listing **本身是合法 MATLAB**,但排版的換行在抽取時
#: 被併掉了,於是變成 `end BETAOLD=BETA;` 與 `if FLAG==1 STEP=2;` ——
#: 兩者在 MATLAB 都是語法錯(`end` 後面要換行或分號,`if` 的條件後面要逗號)。
#: 實測 98 份轉錄有 **96 份** parse error,絕大多數是這兩種。
#:
#: ⚠️ 只補**分隔符**,不動任何 token。這一步不可以改變語意,只能讓斷句回來。
#: ⚠️ `end` 也用在索引(`x(end)`、`Array(end+1)`),但那些後面接的是 `)` 或
#:    `+`,不是識別字 ⇒ 用「end + 空白 + 識別字」當條件就不會誤傷。
RELAYOUT = (
    # end / else 後面直接接語句 ⇒ 補換行
    (re.compile(r"\bend\s+(?=[A-Za-z_])"), "end\n"),
    (re.compile(r"\belse\s+(?=[A-Za-z_])"), "else\n"),
    # if/while 的條件後面直接接賦值 ⇒ 補逗號。
    # ⚠️ `(?!=)` 是要害:沒有它,`if FLAG==1` 的 `==` 會被當成賦值而切錯。
    (re.compile(r"\b(if|while|elseif)\s+([^\n,;]*?)\s+"
                r"(?=[A-Za-z_]\w*\s*=(?!=))"), r"\1 \2, "),
)

# ⚠️⚠️ 這裡**曾經**多一條規則,專治「命令式語句串在一起」
# (`figure plot(...)`、`),grid title(...)`),看起來明顯該加。
# 實測:MATLAB 可解析數從 **12 掉到 0**。已撤除。
#
# 沒有回頭去找它到底切壞了什麼 —— 因為那不重要:重排這一層的價值上限本來就
# 只有「12/98 可解析」,而轉錄品**本來就不保證可跑**(檔頭有寫)。
# 為了追那個上限去疊啟發式,風險是**改變語意而不自知** ——
# 那比語法錯嚴重得多:語法錯會被 MATLAB 擋下,語意錯不會。
#
# ⇒ 判準留在這裡當紀錄:改重排規則之後**一定要重跑 checkcode 計數**,
#   數字沒變好就撤掉。憑「這條規則看起來對」加規則是不行的。

#: 修不掉、必須有人看的形狀。命中就把該 listing 標成未完成。
SUSPECT = (
    (re.compile(r"[¼ðÞþ½]"), "ligature 錯映殘留(¼ð Þþ½)"),
    (re.compile(r"(?<=[\w\)])\s*[,.]=\s*(?=[\w\-.])"), "未還原的 ,= / .="),
    (re.compile(r"\b(if|while|elseif)\s+\w+\s*[,.]\s*[A-Za-z]"), "變數對變數的比較,無法從形狀判斷方向"),
    (re.compile(r"[^\x00-\x7f]"), "其餘非 ASCII"),
)


#: 連續幾行散文才算「離開 listing」。
#: ⚠️ 1 會腰斬(圖說夾在碼中間),太大會把講解段收進來。2 是實測的甜蜜點。
PROSE_RUN = 2


def _prose(line: str) -> bool:
    """這一行看起來是散文而不是程式碼嗎?

    判準刻意保守:寧可多收幾行散文(人看得出來),也不要提早截斷程式碼
    —— 截斷是**靜默**的,而多收不是。
    """
    t = line.strip()
    if not t:
        return False
    # ⭐⭐ 最強的單一訊號:ligature 錯映字元(¼ð Þþ½)**只出現在正文的數學式**,
    #    程式碼區塊是純 ASCII(`XNC=XNP*VC*XLAMD` 用的是真正的 = 與括號)。
    #    所以含它們的行必然不是碼。少了這一條,兩個 listing 之間的式子會被
    #    當成碼、把切點往後推,於是 72/98 被標成「未解決」。
    if re.search(r"[¼ðÞþ½]", t):
        return True
    words = re.findall(r"\b[a-z]{4,}\b", t)
    symbols = sum(t.count(c) for c in "=;()*/")
    # 大量小寫英文單字 + 幾乎沒有運算符號 ⇒ 散文
    return len(words) >= 8 and symbols <= 2


def extract(text: str):
    heads = list(HEAD.finditer(text))
    if not heads:
        raise SystemExit("⚠️ 一個大寫 LISTING 標題都找不到 —— 來源格式變了,"
                         "本檔的錨點失效,不要相信任何輸出。")
    out = []
    for k, m in enumerate(heads):
        stop = heads[k + 1].start() if k + 1 < len(heads) else len(text)
        # ⚠️⚠️ **不靠散文判結束。** 第一版一遇到散文就 break,而圖說會夾在
        #    程式碼中間 ⇒ 靜默截斷(6.2 抽到 2 行、14.1 抽到 1 行,兩個都
        #    「看起來有內容」)。改成:整段收下來,再從**尾端往回**修掉散文。
        #    listing 的排版是「碼在前、講解在後」,所以往回修剪拿得乾淨,
        #    而且不會腰斬。
        # 切點 = 第一段**連續 PROSE_RUN 行**的散文。
        #   * 單行散文不切 —— 那是夾在 listing 中間的圖說/說明,切了就腰斬
        #     (第一版遇散文即 break,6.2 只抽到 2 行、14.1 只抽到 1 行);
        #   * 從尾端往回修剪也不行 —— 兩個 listing 之間隔著好幾頁散文與式子,
        #     而式子含 ¼ð Þ 之類的符號、不被判成散文,往回修剪會提早停下,
        #     於是 34.4 抽出 1159 行(那是後面整本書)。
        body, dropped, run, cut = [], 0, 0, None
        for line in text[m.end():stop].split("\n"):
            if any(p.match(line) for p in NOISE):
                dropped += 1
                continue
            if _prose(line):
                run += 1
                if run >= PROSE_RUN:
                    cut = len(body) - (PROSE_RUN - 1)
                    break
            else:
                run = 0
            body.append(line.rstrip())
        if cut is not None:
            body = body[:max(cut, 0)]
        tail = 0
        # 修剪之後**還夾在中間**的散文要報出來 —— 那是排版異常,不是常態。
        interior = sum(1 for l in body if _prose(l))
        out.append(dict(ch=int(m.group(1)), no=int(m.group(2)),
                        title=" ".join(m.group(3).split()),
                        raw="\n".join(body).strip(),
                        dropped=dropped, tail_prose=tail,
                        interior_prose=interior))
    return out


def repair(raw: str):
    """修 OCR。回傳 (修好的碼, 這次改了什麼, 還有什麼修不掉)。"""
    fixed, log = raw, []
    for bad, good in QUOTES:
        for ch in bad:
            n = fixed.count(ch)
            if n:
                fixed = fixed.replace(ch, good)
                log.append(f"{ch!r}→{good!r} ×{n}")
    for pat, sub in CMP:
        fixed, n = pat.subn(sub, fixed)
        if n:
            log.append(f"{pat.pattern[:28]}… ×{n}")
    # ⚠️ 重排要在比較運算子還原**之後**:`if RTM , 1000 H=.0002` 得先變成
    #    `if RTM < 1000 H=.0002`,重排才知道條件在哪裡結束。
    for pat, sub in RELAYOUT:
        fixed, n = pat.subn(sub, fixed)
        if n:
            log.append(f"relayout {pat.pattern[:20]}… ×{n}")
    left = []
    for pat, why in SUSPECT:
        hits = pat.findall(fixed)
        if hits:
            left.append(f"{why} ×{len(hits)}")
    return fixed, log, left


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="只報告,不寫檔")
    a = ap.parse_args()

    if not SRC.exists():
        print(f"⚠️ 找不到來源 {SRC}")
        return 2
    text = SRC.read_text(encoding="utf-8", errors="replace")
    items = extract(text)

    if not a.dry:
        OUT.mkdir(parents=True, exist_ok=True)

    idx, clean, dirty, tiny = [], 0, 0, 0
    for it in items:
        fixed, log, left = repair(it["raw"])
        lines = [l for l in fixed.split("\n") if l.strip()]
        if it["interior_prose"]:
            left = left + [f"散文夾在程式碼中間 ×{it['interior_prose']} 行"]
        rec = dict(ch=it["ch"], no=it["no"], title=it["title"],
                   file=f"listing_{it['ch']:02d}_{it['no']:02d}.m",
                   n_lines=len(lines), n_chars=len(fixed),
                   noise_dropped=it["dropped"], tail_prose=it["tail_prose"],
                   interior_prose=it["interior_prose"],
                   repairs=log, unresolved=left)
        idx.append(rec)
        # ⚠️ 用**字元數**判過短,不是行數:原文的程式碼擠在超長行裡
        #    (2.1 的整條 data statement 是一行),行數完全沒有鑑別力。
        if len(fixed) < 400:
            tiny += 1
        elif left:
            dirty += 1
        else:
            clean += 1
        if not a.dry:
            hdr = (f"% Zarchan Listing {it['ch']}.{it['no']} — {it['title']}\n"
                   f"%\n"
                   f"% ⚠️ 這是**書上那一頁的轉錄**,不是移植品,不保證可跑。\n"
                   f"%    移植品在 matlab/mgs/+mgs/+zarchan/(那裡才有基準驗證)。\n"
                   f"% ⚠️ 由 matlab/zarchan/extract_listings.py 產生 —— 不要手改這個檔,\n"
                   f"%    要改請改抽取器,否則下次重跑就蓋掉了。\n"
                   f"% 自動修復:{'、'.join(log) if log else '(無)'}\n"
                   f"% 未解決 :{'、'.join(left) if left else '(無)'}\n"
                   f"%\n")
            (OUT / rec["file"]).write_text(hdr + fixed + "\n", encoding="utf-8")

    if not a.dry:
        (OUT / "_index.json").write_text(
            json.dumps(idx, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"來源 {SRC.name}:{len(text):,} 字元")
    print(f"抽出 {len(items)} 個 listing "
          f"—— 乾淨 {clean}、有未解決項 {dirty}、過短可疑 {tiny}\n")
    print(f"{'listing':<10}{'行':>5}{'字元':>8}{'雜訊':>6}  {'狀態':<10}標題")
    print("-" * 104)
    for r in idx:
        st = "過短⚠️" if r["n_chars"] < 400 else ("未解決" if r["unresolved"] else "乾淨")
        print(f"{r['ch']:>2}.{r['no']:<7}{r['n_lines']:>5}{r['n_chars']:>8}"
              f"{r['noise_dropped']:>6}  {st:<10}{r['title'][:52]}")
    bad = [r for r in idx if r["unresolved"] or r["n_chars"] < 400]
    if bad:
        print("\n需要人看的:")
        for r in bad:
            print(f"  {r['ch']}.{r['no']:<3} {r['n_lines']:>4} 行  "
                  f"{'; '.join(r['unresolved']) or '過短'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
