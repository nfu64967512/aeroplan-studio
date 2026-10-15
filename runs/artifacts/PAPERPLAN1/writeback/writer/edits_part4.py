# -*- coding: utf-8 -*-
"""Edit list, part 4 (appendices C-E; none of these are printed in the Word file)."""
from edits_part1 import L

E = []

# ---------------------------------------------------------------- Appendix C
E.append(dict(
    id="E33", serves="圖說：舊圖3（HV4SITL16 內嵌舊值）改為 fig5_compliance（圖3）；fig4_wind 為備用圖4；C1、C2",
    evidence="docs/hv_dccpp/AASRC2026_abstract/fig5_compliance.png、fig4_wind.png（本輪目視核對圖面內容）；figures_src/fig5_data.json、fig4_data.json",
    old=L(292, 296),
    new=(
        "圖說置於圖下緣置中，11 點：\n\n"
        "- 圖1 HARVEST 系統架構與本文 SITL 驗證路徑（粗箭頭）\n"
        "- 圖2 HARVEST 規劃演算法流程（灰底點劃框為不可行或未覆蓋出口，虛線為再分配回圈）\n"
        "- 圖3（檔案 `fig5_compliance.png`）高度合規與實飛間距：(a) 各趟匯出與實飛最高相對高度（HV4SITL15／16 為 120.0 m 天花板且模擬未設風之歷史對照；單變因對照 HV4W55R 見表3），"
        "(b) 規劃最小水平間距（2 s 網格上界）與實飛值（WGS84 橢球連續時間最小值；× 為既有平面近似值）\n"
        "- 備用圖4（檔案 `fig4_wind.png`，本版未排入）風場對照：(a) 巡航段逐航向之（地速 − 空速）中位數，東−西差約為 2W；(b) 規劃風／模擬風組合下之機隊完工時間"
    ),
))

# ---------------------------------------------------------------- Appendix D
E.append(dict(
    id="E34", serves="附錄 D 第 2 項（證據出處）更新：八趟已納管；PAD8、.BIN、基準 CSV、本資料夾仍未納管",
    evidence="`git ls-files`：runs/artifacts/{HV4SITL15,HV4SITL16,HV4W00,HV4W05,HV4W55,HV4W55R,HV4W55S,BEAMIDP1} 各有 27–39 個納管檔；PAD8_BROKEN_RAW、PAD8_FIXED_RAW、docs/hv_dccpp/AASRC2026_abstract/、research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv 皆 0 筆；update_paper/COMMIT_PLAN_ADDENDUM.md §2",
    old=L(301),
    new=(
        "2. **證據出處（v2026-09-21 更新）**：八趟飛行證據（HV4SITL15／16、HV4W00／05／55／55R／55S、BEAMIDP1）已於 95a337b 納管，重算程式與文字包於 3442fcb 納管。仍未乾淨者："
        "(a) 各趟 `.BIN` 未納管，故「.BIN → `out/*.json`」一段只有持有 log 者能重跑；"
        "(b) 表4 兩臂 `PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW` 仍被 `.gitignore:109` 排除（表4 已加註第三方無從核對）；"
        "(c) 附錄 E 表5 航程列之 `project_rows_5e0295f5_dirty.csv` 未納管、`bench.py` 為已修改狀態；"
        "(d) **本資料夾（含本檔與 fig4／fig5）尚未納管**，檔頭版本戳所指 commit 不含本檔；納管後請更新版本戳。"
    ),
))
E.append(dict(
    id="E35", serves="附錄 D 第 4 項（舊「只比門檻多 1.0 m」已過時）＋新增待辦（圖1–3 圖面舊字、[15] 核對、已知缺陷）＋附錄 E（C1–C7 對照、備用 3.6 節／表5＝C5 之主要回答）",
    evidence="NUMBERS.json trials.*.separation；fig1_architecture.png（「含風 Dubins 時間成本」「兩者皆為上界」）、fig2_flowchart.png（「含風時間成本」）、fig3_results.png（101.0 m、0/310）；update_paper/README.md §四；PAPER_TEXT_ZH.md C5、C7-f；TABLES_ZH.md 表 Z；表5 數字同 E31 附錄 B 之表5 列",
    old=L(303),
    new=(
        "4. 最小間距只比門檻多 0.43 m（主結果；本文八趟中最緊 0.29 m），且是 HERO–HERO 同高度帶，最近接時刻垂直差 28.4 m。設風重飛與原生取樣均已完成，量尺更準後餘裕反而變小；"
        "審稿人可能追問，答法：此為情境 `gui_legal_120_1a2b` 本身之性質，天花板改動既未造成也未修好（HV4W55R 為 100.579 m）。\n"
        "5. **圖面上的舊字（本 label 不能改圖）**：`fig3_results.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 m），本版已自排版移除，若要恢復三聯圖須以 HV4W55 重產；"
        "`fig1_architecture.png` 仍寫「含風 Dubins 時間成本」與「覆蓋率與機間間距（兩者皆為上界）」、`fig2_flowchart.png` 仍寫「含風時間成本」，"
        "與正文 C7 用詞（考慮風場之飛行時間成本）及 3.1 節之間距量尺不一致，重產時請改；`fig5_compliance.png` 之 (a) 未含 HV4W55R，若重產建議加入。\n"
        "6. 參考文獻 [15]（Techy & Woolsey 2009）之書目欄位未經 Crossref 逐欄核對；本文依使用者決定將 R_eff 歸於 [15]，投稿前請核對原文確有順風相位曲率半徑之結果，"
        "否則改寫為「由 trochoid[5,15] 之運動學直接可得」。\n"
        "7. `reduce_waypoints()` 之圓弧離散化缺陷未修（本文以 119.5 m 天花板迴避，見限制 (7)）。另 GUI 匯出路徑 `mission/vtol_mission_exporter.py:135` 之 `DO_CHANGE_SPEED` 仍送地速（param1 = 1），"
        "與本文 SITL 路徑（`hv_dccpp/ardupilot.py:653`，空速）不同；論文數據不受影響，但被問及「匯出至 ArduPilot」時須知。\n\n"
        "## 附錄 E　審閱意見 C1–C7 對照與備用內容（給使用者與指導老師，不放進 Word）\n\n"
        "| 意見 | 本版回應位置 | 主要證據 |\n"
        "|---|---|---|\n"
        "| C1 120 m 高度主張與數據不符 | 摘要、前言、2.3（h = 119.5 m）、3.1、3.2、3.3＋表3＋圖3(a)、限制 (7) | `NUMBERS.json` `ceiling_control`；HV4W55 對 HV4W55R |\n"
        "| C2 含風規劃卻在無風 SITL 驗證 | 摘要、2.1、3.1、3.3＋表3、表2 表註、限制 (1)(6)；備用圖4 | `update_paper/out/wind.json`；HV4W00／05／55／55S |\n"
        "| C3 Beam-IDP 跳帶從未觸發 | 2.3（k 值）、3.5、限制 (3)、附錄 A | `prep_beamidp/tables.json`；BEAMIDP1 |\n"
        "| C4 102 項參數之定義 | 摘要、3.2、表2 參數列與表註 | `HV4W55/params_readback.csv`；K2 |\n"
        "| C5 DCCPP 基準較短 | 限制 (4)；完整回答為下方備用 3.6 節＋表5 | `prep_baseline_control/out/stepP_final_table.json`；K9 |\n"
        "| C6 R_eff 未定義 | 前言貢獻 (i)、2.3 式 (4) 與說明、表1、限制 (5)、參考文獻 [15] | `c6_reff/reff_numbers.json`；`hv_dccpp/platform.py:296-304` |\n"
        "| C7「迎風」與實際風三角 | 摘要、前言、2.3 式 (3) 之說明、附錄 A | `hv_dccpp/wind.py:278-306` |\n\n"
        "### 備用 3.6 節（C5 之完整回答；Word 實測有餘時接在 3.5 節之後，須連同表5 與表註整段排入）\n\n"
        "#### 3.6 與既有 DCCPP 實作之對照\n\n"
        "以同一套檢查程式與門檻比較 HARVEST 與團隊既有之 DCCPP 實作（封閉基準；表5）。HARVEST 之最長單機航程較長，換得的是能量可行性與四維排程，代價為起飛佇列。\n\n"
        "**表5 HARVEST 與 DCCPP 基準之能力／KPI 對照（封閉基準）**\n\n"
        "| 指標 | DCCPP 基準 | HARVEST |\n"
        "|---|---|---|\n"
        "| 最長單機航程（10 組） | 較短 | 長 0.91%–22.87%（平均 13.19%） |\n"
        "| 覆蓋率（12 組） | 12／12 組為 1.000 | 10／12 組為 1.000；2 組自判不可行（0.528、0.617） |\n"
        "| 能量可行性（12 組；預算 $D_{model}$ = 97.726 km） | 2／12 組超標（112.5%、114.4%），且規劃器不自報 | 0／12 組超標；自判不可行者恰為該 2 組 |\n"
        "| 四維間隔／起降走廊衝突（10 組） | 114／572 件（重抬升航跡；加上其自帶避撞後仍為 114／572） | 0／0 件（原生四維排程；若重抬升則為 959／688） |\n"
        "| 代價：起飛佇列／最後落地 | 0 s（全機同時起飛） | 330–6460 s／2576.7–9851.3 s |\n\n"
        "表註：衝突數以同一檢查程式與門檻計（$d_{sep}$ = 100 m、$\\Delta h_{sep}$ = 50 m；降落 90 s、起飛 60 s，單一場站），但兩臂之受檢物件不同："
        "基準無四維排程，只能在重抬升航跡（全機 t = 0）上受檢，HARVEST 則在其原生排程上受檢；HARVEST 之 0 來自其自身之四維排程，代價即末列之起飛佇列，本文不主張兩者為同一把尺。"
        "基準有被給予其協調模式（24 組態之任何指標皆零影響）與自帶之事後避撞模組（偵測並處理 753 件後讀數不變），未被給予能量模型，因其程式碼本無此模型。"
        "若改以表1 之可用航程 133 km 為預算，基準 12／12 組皆通過（最緊者 95.4%）；能量估計器對 HARVEST 自身計畫之誤差帶為 −6.86% 至 +2.31%。"
        "航程列之底層資料未納入版本控制，且計算程式 `bench.py` 為已修改狀態，第三方目前無法獨立重現。\n\n"
        "完整之表 2b（七列風場矩陣）、表 2c（天花板對照全欄）、表 X（逐胞）、表 Y、表 Z（含全文圖說）見 "
        "`runs/artifacts/PAPERPLAN1/update_paper/TABLES_ZH.md`；本檔正文表3 與上方表5 為其壓縮版。"
    ),
))
