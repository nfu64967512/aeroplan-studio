# -*- coding: utf-8 -*-
"""Renders CHANGELOG.md for the writer label (called by apply_writeback.py)."""
from __future__ import annotations

import re

DISCREPANCIES = [
    ("HV4W55S 之「東−西」歸零軸區間",
     "PAPER_TEXT_ZH.md §0.4 與 README.md 一.5 寫「−0.032 ~ +0.014」，但同檔 C2-h 逐機列出 −0.032／−0.046／+0.014",
     "NUMBERS.json `trials.HV4W55S.wind_test.null_30m` = −0.0323／−0.046／+0.0139（該趟所用之 30 m 門檻）⇒ 表3 表註 (a) 寫 **−0.046 至 +0.014 m/s**"),
    ("30 m 門檻下 270° 各趟之「東−西」",
     "TABLES_ZH.md 表 2b 表註 (a) 寫「+9.966…+9.977」",
     "NUMBERS.json `trials.{HV4W05,HV4W55,HV4W55R}.wind_test.signal_30m` 實為 9.9488–9.9803 ⇒ 應為 +9.949 至 +9.980；本版**未寫入論文**（表註 (a) 已刪該括號），僅記錄"),
    ("表2 違規取樣分母",
     "TABLES_ZH.md 表 2 寫「違規 0／4 714 對取樣」＝B2–B3 單一機對之取樣數（metrics.json `HV4W55.separation.frames.ecef.pairs.B2-B3.n_samples`）",
     "NUMBERS.json `trials.HV4W55.separation.n_paired_samples` = **7309**（A1–B3 2595 ＋ B2–B3 4714）；違規 0 為全機隊之數，故表2 用 0/7309"),
    ("BEAMIDP1 違規取樣分母",
     "PAPER_TEXT_ZH.md C3-f／TABLES_ZH.md 表 X 腳註寫「違規 0／22 329 對取樣」＝A1–B2 單一機對",
     "NUMBERS.json `trials.BEAMIDP1.separation.n_paired_samples` = 63235；3.5 節未印分母，僅記錄"),
    ("k_bm、k_as 之名稱",
     "PAPER_TEXT_ZH.md C7-g 寫「分別為電池老化與空速裕度係數」",
     "`hv_dccpp/platform.py:951` 註明為「未驗證保守係數」，`:976-979` 在 `battery_monitor_validated`／`airspeed_sensor_validated` 為真時強制 1.0 ⇒ 應為「電池監測器與空速感測器未驗證之保守係數」。C7-g 之句因版面**未寫入正文**；附錄 B 出處列已照程式寫"),
    ("R_eff 可驗證窗口下緣",
     "PAPER_TEXT_ZH.md C6-c 與 README 二.1 寫「W ≳ 5.2 m/s」「5.2–6.15 m/s」",
     "5.2 只對 Pioneer 成立（W* = 5.1675）；HERO 之 W* = 5.0333（`c6_reff/reff_numbers.json`；`c2_wind/evidence/r_eff_and_gust_limits.json` 以格點記為 5.1）⇒ 限制 (5) 寫 **「W* 至 6.15 m/s」**"),
    ("含風規劃晚 47.13 s 之拆解",
     "PAPER_TEXT_ZH.md C2-b 寫「44.0 s 來自延後放行，只有 3.2 s 來自飛得較久」",
     "47.126 − 44.0 = 3.13 s，且 3.2 不在 NUMBERS.json ⇒ 論文只寫 44.0 s（metrics.json 放行 612 → 656 s），**刪去 3.2 s**"),
    ("天花板代價之位數",
     "PAPER_TEXT_ZH.md 第四節結論寫「+0.028 s」",
     "K6 定稿措辭為 +0.0279 s（NUMBERS.json `ceiling_control.planned_makespan_cost_of_1195_s` = 0.027894）⇒ 全文一律 **+0.0279 s**"),
    ("R_eff 之式號",
     "PAPER_TEXT_ZH.md C6-a 說插在「原式 (3) 之前」（會成為式 (3)），但 C7-b 與 TABLES_ZH.md 表 1 表註都寫「式 (4)」",
     "本版把 R_eff 放在地速式 (3) 之後成為 **式 (4)**，與 C7-b、表 1 一致；原 (4)(5) 順延為 (5)(6)"),
    ("固定 skip-k 合計與完工時間推估",
     "TABLES_ZH.md 表 X 寫 1 196.47 s、推估 2 532.6 s",
     "`prep_beamidp/tables.json` 為 skip_t 1196.48、makespan_sequential_estimate_s 2532.5 ⇒ 兩者**皆未寫入論文**，僅記錄"),
    ("表2 參數列用詞",
     "TABLES_ZH.md 表 2 寫「到 float32 精度之原始值**嚴格**比對為 91／96」",
     "K2 定稿措辭不含「嚴格」⇒ 表2 寫「原始值：float32 精度 91／96，逐位元 85／96」"),
    ("版本戳所指 commit 不含本檔",
     "決定 6 之版本戳「對應 commit 3442fcb」",
     "`git ls-files docs/hv_dccpp/AASRC2026_abstract` = 0 筆（整個資料夾未納管）⇒ 版本戳照寫，並於附錄 D 第 2(d) 項載明「commit 不含本檔，納管後請更新」"),
    ("參考文獻 [15] 之內容未核對",
     "決定 2 要求寫明 R_eff 為 [15] 之既有結果",
     "本輪未連網；[15] 之書目與「順風相位曲率半徑 R_theo(1+W/V_a)²」是否出現在其原文均未核對 ⇒ 照決定寫入，列入附錄 D 第 6 項"),
]

NOT_CHANGED = [
    ("題目（主推與兩個備選）、作者、服務單位、關鍵詞", "七項批註皆未觸及"),
    ("0.1（除式號）、0.2 排版約定", "與內容無關；只把式號 (1)…(5) 改為 (1)…(6)"),
    ("2.1 第一、三句；2.2 全段", "無批註觸及；2.1 只在讀回項目加「與模擬風設定」（C2）"),
    ("式 (1)、(2)、原式 (4)(5)（現 (5)(6)）之內容與其 Word 線性格式；常數段（d_thr、Δh_sep、Δt、k_t、σ_max、I(r)）；k_unc 之句", "無批註觸及；只改式號。C7-g 建議補 k_unc 之拆解，因版面未寫入正文（出處在附錄 B）"),
    ("3.4 節（原 3.3）正文與表4 之數值（5.0／19.9 m、5／378；52.7／112.6 m、0／346）", "PAD8 兩臂未納管、本輪未重算（update_paper 不涵蓋 PAD8）；只改表號並在表註加「沿用舊量尺」「未納管、第三方無從核對」（決定 5），數值原樣保留"),
    ("誌謝", "待使用者填寫"),
    ("參考文獻 [1]–[14] 之書目", "[4] 只刪正文中之「4000 m」（決定 3），書目本身未動"),
    ("附錄 A 其餘既有列", "仍正確；「首次…」一列僅補 [15]"),
    ("附錄 D 第 1、3 項", "填表與 HV_TRIAL 警語仍有效"),
    ("`AASRC2026-投稿格式_加長型摘要_.doc`、全部圖檔與 `figures_src/`", "任務規定不碰 .doc；圖檔不在本 label 可改範圍"),
    ("圖面上的舊字：`fig1_architecture.png`「含風 Dubins 時間成本」「（兩者皆為上界）」、`fig2_flowchart.png`「含風時間成本」、`fig3_results.png` 之 101.0 m 等", "不能改圖；圖3 改用 fig5_compliance，圖1／圖2 之用詞問題列入附錄 D 第 5 項待重產"),
]

NOT_PRINTED = [
    "表 2b 全欄（機隊航程、規劃間距、覆蓋率；逐機三元組；HV4SITL15 列）與表 2c 全欄 ⇒ 正文表3 為壓縮版（區間取代逐機值），全欄見 TABLES_ZH.md（附錄 E 指路）",
    "表 X 逐胞表、表 Y ⇒ 3.5 節只寫機隊合計與百分比",
    "表 Z（C5 之主要回答）⇒ 壓縮為表5，放在附錄 E「備用 3.6 節」，Word 實測有餘才排入；正文以限制 (4) 回答 C5。表 Z 第 6、8 列與第 1 列絕對 km 範圍未收",
    "圖4（fig4_wind.png）⇒ 備用，未排入（其數字全在表3）",
    "放行至落地（gate-to-gate）預測誤差 +2.90 ~ +5.94 % 對 +3.11 ~ +4.90 % ⇒ 正文只寫「不比較兩風向之完工時間」",
    "生成位移機制細節（SIM_QuadPlane 地面分支、偏航 127°→174.4°、方位 43.6°–44.4°）⇒ 表3 表註 (b) 保留結論句與「量值未能導出」",
    "BEAMIDP1 之完工時間（K5）、順序掃描完工時間推估（2 532.6 s）、迴轉距離 −33.65%／能量 −32.53%、固定 skip-k 合計 ⇒ 3.5 節只寫迴轉時間 −34.25% 與距 Held–Karp 5.24%",
    "C4-b「關閉整數救援則 100／102」⇒ 只在附錄 B",
    "C7-c 之航向式 ψ = θ − arcsin(W⊥/V_a)、地速上限綁住時逐條帶重選 V_a、`DO_CHANGE_SPEED` 為空速 ⇒ 正文只寫「完整風三角」，出處在附錄 B",
    "C6-b 中「空速座標生成再轉換至地面會成移動目標問題」之理由與 W = 6／8 m/s 之 R_eff 數值 ⇒ 正文只寫「取最壞相位曲率作為地面座標之單一 Dubins 半徑」",
    "C7-d 風三角數值（26.5／16.5／20.9 m/s）⇒ 附錄 A；C7-f GUI 匯出送地速 ⇒ 附錄 D 第 7 項",
    "C1-b「三機總航程 +2.4 m、規劃間距 105.464 → 105.699 m」「3 000 × 3 000 m 幾何上 −21.6 %」⇒ 正文改寫為「此係本作業區之性質，不宜外推」；+2.4 m 與規劃間距變化均未收",
    "C1-c「距法規 121.92 m 餘裕 2.245 m」與逐機最高高度文字 ⇒ 逐機值在表2",
    "C2-b 之 EKF3 風向 270.0°–270.5°、180° 之 180.1°–180.4°、東−西 +9.915 至 +9.969 之文字版 ⇒ 表3 帶區間，文字只引 EKF3 風速",
    "限制 (6) 之「未採用[5]之垂直於風掃描」、限制 (1) 之「陣風取 1.3 倍」細節 ⇒ 分別刪去與移至表3 表註 (c)",
    "K8 電池遙測 ⇒ 只在附錄 A（論文正文原本就沒有這句）",
]

STALE_PATTERNS = [
    ("貼齊", "舊主張「高度貼齊 120 m 上限」"),
    ("迎風", "C7 舊用詞"),
    ("含風時間成本", "C7 舊用詞（成本名稱）"),
    ("含風有效", "C6／C7 舊用詞"),
    ("101.0 m", "K1 舊間距"),
    ("104.2", "K1 舊重飛間距"),
    ("1.248 km", "決定 1 舊分母"),
    ("少於 3", "C3 錯誤門檻"),
    ("<3", "C3 錯誤門檻（附錄 A 舊寫法）"),
    ("138.6", "C1-f 舊刈幅"),
    ("97.0 m", "C1-f 舊間距"),
    ("HV4SITL16", "舊主結果架次"),
    ("+0.02 s", "K6 四捨五入假影"),
    ("4000 m", "決定 3"),
    ("0.008 m/s", "K7 舊值"),
    ("68/68", "舊參數數"),
    ("120.10", "舊最高高度"),
    ("0/310", "舊違規分母"),
    ("未設風", "C2：只能出現在描述歷史對照／BEAMIDP1 之處"),
]


def _fence(s: str) -> str:
    return "~~~~\n" + s + "\n~~~~"


def stale_report(after: str) -> str:
    lines = after.split("\n")
    i_app = next(i for i, x in enumerate(lines) if x.startswith("## 附錄 A"))
    i_body = next(i for i, x in enumerate(lines) if x.startswith("## 1. 封面資訊"))
    out = []
    for pat, why in STALE_PATTERNS:
        hits = [(i + 1, x) for i, x in enumerate(lines) if pat in x]
        body = [h for h in hits if i_body <= h[0] - 1 < i_app]
        out.append(f"### `{pat}`（{why}）：全檔 {len(hits)} 處，其中印刷區（第 1、2 節）{len(body)} 處")
        for n, x in hits:
            j = x.find(pat)
            ctx = x[max(0, j - 40): j + len(pat) + 40].replace("|", "¦")
            where = "印刷區" if i_body <= n - 1 < i_app else ("附錄" if n - 1 >= i_app else "第 0 節排版指示（不印）")
            out.append(f"- L{n}（{where}）：…{ctx}…")
        out.append("")
    return "\n".join(out)


def render(edits, before, after, est_b, est_a, scen, sha_b, sha_a) -> str:
    out = []
    out.append("# CHANGELOG — label `writer`：審閱意見 C1–C7 寫回 `cowork_brief.md`\n")
    out.append("- 目標檔：`docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md`（本 label 唯一修改之專案檔）")
    out.append("- 備份：`runs/artifacts/PAPERPLAN1/writeback/cowork_brief.before.md`")
    out.append(f"- sha256 before `{sha_b}`")
    out.append(f"- sha256 after  `{sha_a}`")
    out.append("- 重現：`python runs/artifacts/PAPERPLAN1/writeback/writer/apply_writeback.py`（只讀備份、冪等；每筆 `old` 必須恰好出現一次，否則中止）。"
               "編輯清單在同目錄 `edits_part1.py`–`edits_part4.py`，本檔由 `changelog_text.py` 自同一份清單產生。")
    out.append("- 本 label **沒有** commit、沒有碰 `.doc` 與圖檔、沒有飛 SITL、沒有 COM 埠、沒有 pip install。\n")

    out.append("## 一、長度估計（⚠ 4 頁限制目前**無法**只靠 Cowork 可執行的刪減達成）\n")
    out.append("| | v2026-09-14（before） | v2026-09-21（after） |")
    out.append("|---|---|---|")
    out.append(f"| 檔案行數 | {before.count(chr(10))} | {after.count(chr(10))} |")
    out.append(f"| 檔案字元數 | {len(before)} | {len(after)} |")
    out.append(f"| 印刷區（第 1、2 節）原始字元數 | {est_b['printed_chars']} | {est_a['printed_chars']} |")
    out.append(f"| 估計印刷行數（不含圖） | {est_b['text_lines']} | {est_a['text_lines']} |")
    out.append(f"| 排入之圖（總高 cm） | {', '.join(est_b['figures'])}（{est_b['figure_cm']}） | {', '.join(est_a['figures'])}（{est_a['figure_cm']}） |")
    out.append(f"| **估計頁數（取消文件格線）** | **{est_b['pages_no_grid']}** | **{est_a['pages_no_grid']}** |")
    out.append(f"| 估計頁數（未取消格線，18 pt／行） | {est_b['pages_with_grid']} | {est_a['pages_with_grid']} |")
    out.append(f"| 依各自之 0.4 刪減順序做完（Cowork 可執行者） | {scen['before_after_its_own_cut_items_1_to_3']} | {scen['after_cut_items_1_to_5']} |")
    out.append(f"| 再加 0.4 第 6–8 項（須使用者決定） | — | {scen['after_cut_items_1_to_8']} |")
    out.append("")
    out.append("逐項累積（本版，於寫出之檔案上模擬 0.4 節各項）：" + "；".join(
        f"做到第 {n} 項 {scen[f'after_cut_items_1_to_{n}']} 頁" for n in range(1, 9)) + "。\n")
    out.append("各節估計行數（after）：" + "、".join(f"{k} {v}" for k, v in est_a["lines_by_section"].items()) + "。\n")
    out.append("**估計法**（`apply_writeback.py:estimate`）：只計第 1、2 節中會印出的內容（排除 Cowork 指示、Word 線性格式、備選題目）；"
               "中文與全形字 1 單位、ASCII 0.5 單位；每行 43.8 單位（17 cm ÷ 11 pt）；段落與表格儲存格各自向上取整；"
               "取消格線時每行 0.494 cm（約 14 pt），未取消時 0.635 cm（18 pt）；圖高以 PNG 像素 ÷ 300 dpi 實算（圖1 6.80、圖2 7.59、圖3 6.20 cm）；A4 版心高 25.7 cm。"
               f"**校準**：同法估 v2026-09-14 稿未取消格線為 {est_b['pages_with_grid']} 頁，而 0.4 節之實測為「約 6 頁」⇒ 本估計法**偏低約 15%**，"
               "實際頁數應再往上修。這是估計，不是 Word 實測。\n")
    out.append(f"**結論**：v2026-09-14 稿依它自己的刪減順序估計 {scen['before_after_its_own_cut_items_1_to_3']} 頁（以偏低 15% 校正約 {scen['before_after_its_own_cut_items_1_to_3']*1.15:.1f} 頁，已接近 4 頁），本版依 1–5 項估計 {scen['after_cut_items_1_to_5']} 頁（校正約 {scen['after_cut_items_1_to_5']*1.15:.1f} 頁）。"
               "三輪壓縮後（第一輪原樣套入文字包估計約 8 頁），淨增主要來自 3.3 節＋表3（C1、C2）、2.3 節之式 (4) 與表1 新列（C6）、3.2 節參數定義（C4）、3.5 節（C3）與擴寫之限制。"
               "0.4 節已把刪減分成兩級：1–5 項 Cowork 可直接做；6–8 項（圖1 改 compact 版、刪圖2、3.4 節與表4 改一句）會刪掉實質內容，須由使用者決定。"
               f"即使 1–8 全做，估計仍約 {scen['after_cut_items_1_to_8']} 頁（且估計法偏低），**需要使用者再決定實質取捨**（例如前言相關文獻段縮為一句、或把 3.5 節併入限制 (3)），本 label 不代做。\n")

    out.append("## 二、逐筆修改（依套用順序）\n")
    for e in edits:
        out.append(f"### {e['id']}　{e['serves']}\n")
        out.append(f"**證據**：{e['evidence']}\n")
        out.append("**舊文**：\n")
        out.append(_fence(e["old"]))
        out.append("\n**新文**：\n")
        out.append(_fence(e["new"]))
        out.append("")
    out.append("> 註：E06 新文中的 `__PAGES_*__` 於寫檔時以第一節的估計頁數取代（見寫出之檔案 0.4 節）。\n")

    out.append("## 三、數字不一致（以 NUMBERS.json 或原始證據為準）\n")
    out.append("| 項目 | 文字包原寫 | 本版採用與依據 |")
    out.append("|---|---|---|")
    for a, b, c in DISCREPANCIES:
        out.append(f"| {a} | {b} | {c} |")
    out.append("")

    out.append("## 四、刻意**未改**之處與理由\n")
    out.append("| 位置 | 理由 |")
    out.append("|---|---|")
    for a, b in NOT_CHANGED:
        out.append(f"| {a} | {b} |")
    out.append("")

    out.append("## 五、文字包中**未放進印刷正文**的內容（4 頁限制；仍可在 TABLES_ZH.md／PAPER_TEXT_ZH.md 或本檔附錄找到）\n")
    for x in NOT_PRINTED:
        out.append(f"- {x}")
    out.append("")

    out.append("## 六、舊主張殘留檢查（寫檔後逐字 grep；「印刷區」＝第 1、2 節，會進 Word）\n")
    out.append("預期：印刷區內之殘留只能是 (i) 歷史對照架次 HV4SITL16 之列與表註、(ii) 對 BEAMIDP1／HV4SITL16「未設風」之揭露、(iii) 限制 (3) 中以否定句出現的「少於 3」、"
               "(iv) 表2 表註中「平面近似之 1.248 km² 高估 0.37%」之說明；其餘出現處均在附錄 A（禁用欄）、附錄 B（出處）或附錄 D（待辦）。\n")
    out.append(stale_report(after))
    return "\n".join(out) + "\n"
