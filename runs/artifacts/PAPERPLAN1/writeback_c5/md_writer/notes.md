# md_writer 任務筆記（C5 四頁版寫回 cowork_brief.md）

## 做了什麼

1. `docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md` 於編輯前先原樣複製到：
   - `docs/hv_dccpp/AASRC2026_abstract/cowork_brief_long.md`（使用者保留之完整版）
   - `runs/artifacts/PAPERPLAN1/writeback_c5/cowork_brief.before_c5.md`（本次任務的编辑前快照）
   三檔於複製當下逐位元相同（已用 `diff` 確認）。

2. 用 Python 直接呼叫排版工具本身的模組（`paper_model.py`、`candidates.py`、`candidates_extra.py`）——**不修改**這些模組——對**編輯前**的 `cowork_brief.md` 依序套用：
   - `paper_model.apply_cuts(blocks, [1,2,3,4,5,6,7,8])`（0.4 節刪減 1–8）
   - `candidates_extra.REPORT_COMBOS["C5 四頁(須刪圖1與圖3)"]` 所列全部候選（依 `candidates_extra.ORDER` 排序套用）：
     `L1, K1, K4, K5, K7, K8, K10, K11, K12, F1, F2, P1, P2, P4, P5, L2, L3, R1, R2, D1, D2, E1`

   得到的區塊清單（61 個區塊）即「C5 應該長什麼樣子」的機器可讀真值（ground truth），並逐字核對與
   `runs/artifacts/PAPERPLAN1/writeback_c5/C5_reference.txt` 相符（用 Word Content.Text 抽出比對，見下）。

3. 依照這份真值，把每個區塊的**確切替換文字**（候選函式的 `text`／`replacement_text_md`，與 `results/summary.json`
   內同名欄位逐字相同）直接手工寫回 `cowork_brief.md` 的「## 1. 封面資訊」與「## 2. 正文」兩節，取代原本
   entrusted 給排版工具在執行期動態套用的邏輯。表號／圖號依刪減後的最終編號手動改寫（例如原表2→表1、原表3→表2；
   原圖1／圖2／圖3全部移除）。L1、L2、L3（式(4)兩行排版、表註10點、表格可跨頁）**沒有改動文字**，只記錄成
   0.2 節的排版規則，符合任務指示。

4. 更新非印刷部分：
   - 版本戳（v2026-09-21b）：加註第 1–2 節已套用 C5 版全部刪減與候選，並指向 `cowork_brief_long.md`。
   - 0.2 節：新增 L1／L2／L3 為已定案之排版規則。
   - 0.3 節：頁面配置表改用 `results/C5.json` 的 `where` 實測逐頁位置（非估計值）。
   - 0.4 節：全面改寫為「已套用」記錄，逐項對照原 1–8 與所加候選，並指向 `cowork_brief_long.md`。
   - 附錄 C／D／E：在描述已移除之圖（圖1／圖2／圖3）或舊節號（2.3、3.4、3.5、原表3／表4）之處，加註「本節/本項
     適用於 `cowork_brief_long.md`」的提示，不改動其中任何數字。

## 自測（第 4 步）

用工具本身跑：

```
python run_variants.py "V0+L1+L2+L3"
```

（V0 = 不套用任何刪減；候選僅 L1、L2、L3，皆為版面規則）於**編輯後**的 `cowork_brief.md` 上，產出
`variants/V0pL1pL2pL3.docx` 與 `results/V0pL1pL2pL3.json`（已複製一份到本資料夾）。

量頁結果與官方 C5 組合（`python run_variants.py "C5 四頁(須刪圖1與圖3)"`，其量頁存於
`word_layout/results/C5.json`）**完全一致**：4 頁、`content_cm` 均為 98.97、`used_cm_per_page` 均為
`[25.38, 25.57, 25.25, 22.77]`、`where`（各節/表起始頁與位置）逐項相同、`n_omath` 皆為 4、`caveats_missing`
皆為空、`figures` 皆為空。

文字比對：用私有、唯讀的 `DispatchEx("Word.Application")`（`extract_text.py`，只開自己複製的 .docx、只 Quit
自己的執行個體）讀出 `Content.Text`，只把 `\r`（不含表格列尾標記 `\x07`）換成 `\n`，與抽取
`C5_reference.txt` 用的方法一致。抽出文字存於 `V0pL1pL2pL3_extracted.txt`；與 `C5_reference.txt` 之逐行差異
見 `V0pL1pL2pL3_vs_C5_reference.diff`。

## 結果：僅剩一個已知且無法消除的差異

`diff --strip-trailing-cr` 顯示全文 405 行中僅 4 行不同，且全部集中在 2.2 節保留之 4 條顯示方程式的**式號**：

| 位置（式子） | 本檔單獨排版（V0+L1+L2+L3） | 官方 C5 組合 |
|---|---|---|
| 地速 $V_g(\theta)$ | (3) | (1) |
| $R_{eff}$ | (4) | (2) |
| $\lambda_{floor}$ | (5) | (3) |
| 四維隔離 conflict/site | (6) | (4) |

**原因（工具本身的限制，非本檔寫法問題）**：`paper_model.py` 剖析 `$$...\tag{N}$$` 時，`N` 同時決定兩件事：
1. 到 `EQ_LINEAR` 字典（`{"1":…makespan, "2":…swath, "3":…velocity, "4":…R_eff, "5":…λ_floor, "6":…conflict}`）
   查表得到的**公式內容**；
2. 剖析當下直接指派的**顯示式號**（`num = f"({N})"`）。

`EQ_LINEAR` 只有鍵 `"1".."6"`／`"6b"`，且每個鍵綁定固定內容，所以本檔要讓「地速公式」正確顯示，其
`\tag{}` 必須是 `3`（否則查到錯的公式内容）；但只要 `\tag{}` 是 `3`，剖析出的式號就一定是 `(3)`，不會是 `(1)`。
把 (3)(4)(5)(6) 改標成 (1)(2)(3)(4) **的唯一機制**是 `D2` 候選函式本身（`candidates_extra.c_eq12_prose` 裡的
`ren = {"(3)":"(1)", "(4)":"(2)", "(5)":"(3)", "(6)":"(4)"}`，只改 `b["num"]`、不改 `b["linear"]`／`\tag`），
而這個轉換**依賴一個已刪除的區塊組合**（原式(1)(2)加上原「規劃目標為最小化機隊完工時間…」段落）才能定位要
刪併的區塊；本檔第 2.2 節已經是刪併後的最終文字，若對本檔重跑 `D2`，其 `assert`／索引搜尋會直接失敗
（找不到式號 `(1)` 的 eq 區塊），不是「重跑就會修好」。

換言之：**這是 `paper_model.py` 把「公式內容」與「初次剖析時的顯示式號」耦合在同一個 `\tag{}` 數字上所致**，
純粹修改 `.md` 文字無法解開這個耦合；要解開必須修改 `paper_model.py`（例如讓 `\tag{}` 可與顯示式號分離），而
任務規則明確禁止修改排版工具程式碼，故本次以「保留正確公式內容、式號維持原始 3/4/5/6、並在 0.4 節與本檔逐字
記錄原因與影響範圍」處理。

**這個差異只出現在「單獨對本檔套用 V0+L1+L2+L3」時**；官方交付路徑是
`python run_variants.py "C5 四頁(須刪圖1與圖3)"`（cuts 1–8 + 完整候選清單，含 `D2`），對**原始未刪減**的
`cowork_brief_long.md` 執行，其輸出即 `C5_reference.txt`，式號正確為 (1)(2)(3)(4)。也就是說：Word 稿本身
（`C5_reference.txt` 或用官方組合重新排出的 .docx）不受影響；受影響的只是「以後若只想用 V0+L1+L2+L3 這條更
簡單的路徑排版」這個情境下，4 條方程式的式號會停在 (3)(4)(5)(6)。

## 文字是否逐字相同

除上述 4 處式號外，**逐字相同**：標題、作者、摘要、關鍵詞、前言、2.1／2.2 節全部內文、4 條方程式本身的公式
內容、表1（原表2）全部儲存格與表註、表2（原表3）全部儲存格與表註（已刪 HV4SITL16 與 W≥6.16 兩列）、3.1／
3.2／3.3 節內文、結論段（含併入之原 3.4 節內容）、限制 (1)–(5)（含併入之原 3.5 節內容於限制(3)）、誌謝、
參考文獻 15 條（已套用 R1 只列年份、R2 期刊 ISO 4 縮寫）。頁數為 4 頁，與 `C5_reference.txt` 之頁數一致。

## 檔案清單（本資料夾）

- `notes.md` — 本檔。
- `V0pL1pL2pL3_extracted.txt` — 用 `V0+L1+L2+L3` 對編輯後 `cowork_brief.md` 排版所得 .docx 之 `Content.Text`
  抽取結果（方法與 `C5_reference.txt` 相同）。
- `V0pL1pL2pL3_vs_C5_reference.diff` — 上述抽取結果與 `C5_reference.txt` 的逐行差異（僅 4 行，見上表）。
- `V0pL1pL2pL3.json` — `run_variants.py` 對本次排版的量頁結果（`word_layout/results/V0pL1pL2pL3.json` 之副本），
  與 `word_layout/results/C5.json` 之 `pages`／`content_cm`／`used_cm_per_page`／`where`／`n_omath`／
  `caveats_missing` 逐項相同。

## 未改動之處（依任務規則）

僅編輯 `cowork_brief.md`；新建 `cowork_brief_long.md`（docs 資料夾）與本 `writeback_c5/` 下的檔案。未改動
`.doc` 格式檔、任何圖檔、`word_layout/` 下的任何 `.py`（`paper_model.py`／`candidates.py`／
`candidates_extra.py`／`candidate_texts.py`／`run_variants.py`／`render_xml.py` 等）。未執行任何 git 操作。
Word 自動化僅用 `DispatchEx` 私有執行個體，讀取／關閉／Quit 皆僅限本次自己開啟的檔案。
