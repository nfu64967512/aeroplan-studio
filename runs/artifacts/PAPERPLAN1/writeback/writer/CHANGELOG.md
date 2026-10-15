# CHANGELOG — label `writer`：審閱意見 C1–C7 寫回 `cowork_brief.md`

- 目標檔：`docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md`（本 label 唯一修改之專案檔）
- 備份：`runs/artifacts/PAPERPLAN1/writeback/cowork_brief.before.md`
- sha256 before `f0ccb7fdc432647e211c78a39c5ac2bfc26ce66adbb2812b7979cc9f6d38887f`
- sha256 after  `05d8ee956affdd53f88ae099210afedebfc9a2be724e75533db896f54548b6ed`
- 重現：`python runs/artifacts/PAPERPLAN1/writeback/writer/apply_writeback.py`（只讀備份、冪等；每筆 `old` 必須恰好出現一次，否則中止）。編輯清單在同目錄 `edits_part1.py`–`edits_part4.py`，本檔由 `changelog_text.py` 自同一份清單產生。
- 本 label **沒有** commit、沒有碰 `.doc` 與圖檔、沒有飛 SITL、沒有 COM 埠、沒有 pip install。

## 一、長度估計（⚠ 4 頁限制目前**無法**只靠 Cowork 可執行的刪減達成）

| | v2026-09-14（before） | v2026-09-21（after） |
|---|---|---|
| 檔案行數 | 303 | 417 |
| 檔案字元數 | 15419 | 28859 |
| 印刷區（第 1、2 節）原始字元數 | 9370 | 13808 |
| 估計印刷行數（不含圖） | 176.6 | 265.7 |
| 排入之圖（總高 cm） | fig1_architecture.png, fig2_flowchart.png, fig3_results.png（20.6） | fig1_architecture.png, fig2_flowchart.png, fig5_compliance.png（20.6） |
| **估計頁數（取消文件格線）** | **4.2** | **5.91** |
| 估計頁數（未取消格線，18 pt／行） | 5.16 | 7.37 |
| 依各自之 0.4 刪減順序做完（Cowork 可執行者） | 3.85 | 5.37 |
| 再加 0.4 第 6–8 項（須使用者決定） | — | 4.91 |

逐項累積（本版，於寫出之檔案上模擬 0.4 節各項）：做到第 1 項 5.81 頁；做到第 2 項 5.58 頁；做到第 3 項 5.5 頁；做到第 4 項 5.45 頁；做到第 5 項 5.37 頁；做到第 6 項 5.31 頁；做到第 7 項 4.99 頁；做到第 8 項 4.91 頁。

各節估計行數（after）：封面 4.3、摘要 15.0、一、前言 16.5、二、研究方法 1.5、2.1 系統架構 6.8、2.2 規劃流程 5.8、2.3 核心公式 51.8、三、SITL 驗證與成果 1.5、3.1 實驗設置 10.3、3.2 規劃與 SITL 25.3、3.3 規劃天花板與風場 38.8、3.4 起降走廊判準對照 12.3、3.5 跳帶排序之觸發情 7.3、四、結論與限制 21.5、誌謝 2.5、參考文獻 44.5。

**估計法**（`apply_writeback.py:estimate`）：只計第 1、2 節中會印出的內容（排除 Cowork 指示、Word 線性格式、備選題目）；中文與全形字 1 單位、ASCII 0.5 單位；每行 43.8 單位（17 cm ÷ 11 pt）；段落與表格儲存格各自向上取整；取消格線時每行 0.494 cm（約 14 pt），未取消時 0.635 cm（18 pt）；圖高以 PNG 像素 ÷ 300 dpi 實算（圖1 6.80、圖2 7.59、圖3 6.20 cm）；A4 版心高 25.7 cm。**校準**：同法估 v2026-09-14 稿未取消格線為 5.16 頁，而 0.4 節之實測為「約 6 頁」⇒ 本估計法**偏低約 15%**，實際頁數應再往上修。這是估計，不是 Word 實測。

**結論**：v2026-09-14 稿依它自己的刪減順序估計 3.85 頁（以偏低 15% 校正約 4.4 頁，已接近 4 頁），本版依 1–5 項估計 5.37 頁（校正約 6.2 頁）。三輪壓縮後（第一輪原樣套入文字包估計約 8 頁），淨增主要來自 3.3 節＋表3（C1、C2）、2.3 節之式 (4) 與表1 新列（C6）、3.2 節參數定義（C4）、3.5 節（C3）與擴寫之限制。0.4 節已把刪減分成兩級：1–5 項 Cowork 可直接做；6–8 項（圖1 改 compact 版、刪圖2、3.4 節與表4 改一句）會刪掉實質內容，須由使用者決定。即使 1–8 全做，估計仍約 4.91 頁（且估計法偏低），**需要使用者再決定實質取捨**（例如前言相關文獻段縮為一句、或把 3.5 節併入限制 (3)），本 label 不代做。

## 二、逐筆修改（依套用順序）

### E01　決定 6（版本戳）

**證據**：使用者決定；`git log` 3442fcb／95a337b／ff38a77

**舊文**：

~~~~
# AASRC 2026 加長型摘要 — 給 Cowork 的排版素材包（HARVEST，中文稿）

> 本檔是
~~~~

**新文**：

~~~~
# AASRC 2026 加長型摘要 — 給 Cowork 的排版素材包（HARVEST，中文稿）

**v2026-09-21，對應 commit 3442fcb（證據 95a337b、腳本 ff38a77）**

> 本檔是
~~~~

### E02　通則（數字來源改為 NUMBERS.json）

**證據**：runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json

**舊文**：

~~~~
（出處見附錄 B）。**請勿改動數字
~~~~

**新文**：

~~~~
（出處見附錄 B；本版依指導老師審閱意見 C1–C7 改寫，飛行數字一律以 `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json` 為準）。**請勿改動數字
~~~~

### E03　C1／K1（舊圖3 內嵌已更正之舊值）＋本輪新圖 fig5／fig4

**證據**：docs/hv_dccpp/AASRC2026_abstract/fig3_results.png（圖內印有 最小 101.0 m、違規 0/310、飛行最高 120.10/120.12/120.16 m、「法規上限 120 m，不豁免」）；figures_src/fig{3,4,5}_data.json；fig5_compliance.png 2007×732 px（17.0 × 6.2 cm）、fig4_wind.png 2007×708 px（17.0 × 6.0 cm）

**舊文**：

~~~~
| `fig3_results.png`（.svg） | 圖3 SITL 飛行結果，300 dpi，寬 17.0 cm |
| `figures_src/` | 產圖腳本與圖3 數據出處（`fig3_data.json`），不放進 Word |
~~~~

**新文**：

~~~~
| `fig5_compliance.png`（.svg） | **圖3** 高度合規與實飛間距，300 dpi，寬 17.0 cm（檔名為 fig5，於正文為圖3） |
| `fig4_wind.png`（.svg） | **備用圖4** 風場對照，300 dpi，寬 17.0 cm；**本版未排入**，Word 實測有餘時才接在圖3 之後（見 0.4） |
| `fig3_results.png`（.svg） | ⚠ **本版不放進 Word**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更正（見附錄 D） |
| `figures_src/` | 產圖腳本與數據出處（`fig3_data.json`、`fig4_data.json`、`fig5_data.json`），不放進 Word |
~~~~

### E04　C6（新增式 (4)，式號順延）

**證據**：本檔 2.3 節

**舊文**：

~~~~
式號 (1)…(5) 靠右
~~~~

**新文**：

~~~~
式號 (1)…(6) 靠右
~~~~

### E05　版面配置（新增 3.3、3.5 節與表3）

**證據**：本檔第三節

**舊文**：

~~~~
| 第 3 頁 | 2.3 核心公式 (1)–(5)＋表1；三、SITL 驗證：3.1 設置、3.2 結果＋表2 |
| 第 4 頁 | 圖3；3.3 起降走廊判準對照＋表3；四、結論與限制；誌謝；參考文獻 |
~~~~

**新文**：

~~~~
| 第 3 頁 | 2.3 核心公式 (1)–(6)＋表1；三、SITL 驗證：3.1 設置、3.2 主結果＋表2 |
| 第 4 頁 | 3.3 天花板與風場對照＋表3＋圖3；3.4 起降走廊判準對照＋表4；3.5 跳帶排序觸發情境；四、結論與限制；誌謝；參考文獻 |
~~~~

### E06　4 頁限制（刪減順序改寫，附估計頁數）

**證據**：runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md 第一節與 length_estimate.json

**舊文**：

~~~~
> ⚠ 實測：本稿原樣以 Word 套官方格式檔排版、**未取消文件格線**時約 **6 頁**。取消格線（0.2 節）後的頁數尚未量測，預期仍可能超過 4 頁，請先取消格線，再依下列順序刪減。

1. 2.1、2.2 節文字各只保留第一句（「系統架構如圖1。」「規劃流程如圖2。」後接圖）。
2. 表1 改成內文一句話：「Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m，可用航程 133–145／79–86 km，兩型共用 HFOV 60° 相機。」
3. 3.3 節刪文字，只留表3 與表註。

**不可刪**：摘要、表2、圖3、第四節的「限制」、所有表註（表註是 caveat，刪了就是誤導）。
~~~~

**新文**：

~~~~
> ⚠ 實測：v2026-09-14 稿原樣以 Word 套官方格式檔排版、**未取消文件格線**時約 **6 頁**；取消格線（0.2 節）後的頁數尚未量測。本版（v2026-09-21）為回應審閱意見 C1–C7 新增 3.3、3.5 節與表3，以字數估計取消格線後約 __PAGES_AFTER__ 頁（同法估 v2026-09-14 稿約 __PAGES_BEFORE__ 頁，該稿依其刪減順序做完約 __PAGES_BEFORE_CUT__ 頁；估計法見 `runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md`；此估計法以舊稿實測校準約偏低 15%）。請先取消格線，再依下列順序刪減，直到 4 頁為止。

1. 2.1、2.2 節文字各只保留第一句（「系統架構如圖1。」「規劃流程如圖2。」後接圖）。
2. 表1 改成內文一句話：「Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m（規劃裕度可吸收之風速 W* = 5.1675／5.0333 m/s），可用航程 133–145／79–86 km（估計值），兩型共用 HFOV 60° 相機。」表1 之表註保留，接在該句之後。
3. 3.4 節刪文字，只留表4 與表註。
4. 3.5 節只留「另設觸發情境…均為規劃層模型值。」一句。
5. 表3 刪 HV4SITL16 與「W ≥ 6.16 m/s」兩列（表註 (c)(d) 保留）。

⚠ 1–5 項全做後估計仍約 __PAGES_CUT__ 頁。**以下 6–8 項會刪掉實質內容，須由使用者決定，Cowork 不可自行執行**（全做後約 __PAGES_CUT8__ 頁）：

6. 圖1 改用同資料夾之 `fig1_architecture_compact.png`（同內容，高 5.3 cm；原圖 6.8 cm）。
7. 刪圖2 與其圖說，並把 2.2 節首句「規劃流程如圖2。」改為「規劃流程如下。」（2.2 節其餘文字保留）。
8. 3.4 節與表4 改為一句：「另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定則為 52.7 m 且無違規，而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管）。」

**不可刪**：摘要、表2、表3、圖3、第四節的「限制」、所有表註（表註是 caveat，刪了就是誤導）。備用圖4（`fig4_wind.png`）與附錄 E 之備用 3.6 節（表5）只有在 Word 實測仍不足 4 頁時才排入。
~~~~

### E07　C1-a（刪「貼齊 120 m」）、C4-a＋決定 4、C7-a（用詞）、決定 1（1.2434 km²）、K1（100.43 m）、C2（主結果改為設風架次 HV4W55）、C1-f（間距 96.6 m）

**證據**：NUMBERS.json trials.HV4W55（coverage 0.999666、flown_max_agl 119.6752、min_sep 100.4256、params n_ok 102/102）；metrics.json HV4W55.timing.per_uav（dist_err 3.813／4.675／3.736%）；NUMBERS.json ceiling_control（t_above_120 354.925 s）；survey_geometry（area_km2 1.243447、spacing 96.5907 m）

**舊文**：

~~~~
本文針對法規高度上限下之異質垂直起降（VTOL）無人機隊測繪任務，提出 HARVEST 規劃流程。在 120 m 作業高度下，小型複合翼 VTOL 之轉彎半徑（120–200 m）與條帶間距（約 97 m）同量級，且每架次皆含起降與場坪排隊之固定成本。HARVEST 結合含風時間成本、依轉彎半徑與可用能量之決策級機數配額、Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離產生任務，並匯出至 ArduPilot QuadPlane。以 1 架 Pioneer 與 2 架 HERO 覆蓋 1.248 km² 作業區進行 SITL 閉迴路飛行，三機皆完成起降，航跡回算覆蓋率 100%（上界），最高高度貼齊 120 m 上限（最大超出 0.16 m），航程較規劃多 2.3%，最小水平間距 101.0 m（門檻 100 m），飛控參數讀回 102/102 相符。另一受控對照顯示，走廊豁免若以場坪座標而非場坪身分判定，最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。
~~~~

**新文**：

~~~~
本文針對法規高度上限下之異質垂直起降（VTOL）無人機隊測繪任務，提出 HARVEST 規劃流程。在 120 m 作業上限下，小型複合翼 VTOL 之轉彎半徑（120–200 m）與條帶間距（約 96.6 m）同量級，且每架次皆含起降與場坪排隊之固定成本。HARVEST 結合考慮風場之飛行時間成本、依轉彎半徑與可用能量之決策級機數配額、Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離產生任務，並匯出至 ArduPilot QuadPlane。以 1 架 Pioneer 與 2 架 HERO 覆蓋 1.2434 km² 作業區，於 5 m/s 模擬風場中進行 SITL 閉迴路飛行，三機皆完成起降，航跡回算覆蓋率 99.97%（上界），全程最高高度 119.68 m AGL，低於 120 m 上限（規劃天花板 119.5 m），航程較規劃多 3.7%–4.7%，最小水平間距 100.43 m（門檻 100 m），飛控參數讀回與韌體正規化後之期望值 102/102（34 項 × 3 機）相符。單變因對照顯示，規劃天花板若取 120.0 m，實飛累計 354.93 s 高於 120 m。另一受控對照顯示，走廊豁免若以場坪座標而非場坪身分判定，最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。
~~~~

### E08　C1-f（刈幅／間距隨 119.5 m 天花板連動）

**證據**：NUMBERS.json survey_geometry.swath_and_spacing['119.5'] = 137.9867／96.5907 m

**舊文**：

~~~~
本文取 120 m 為上限；此時相機刈幅約 138.6 m、條帶間距約 97.0 m，
~~~~

**新文**：

~~~~
本文取 120 m 為作業上限、119.5 m 為規劃天花板；於後者，相機刈幅約 138.0 m、條帶間距約 96.6 m，
~~~~

### E09　決定 3（[4] 之 4000 m 未核對，刪數字）＋C7-a（用詞統一）

**證據**：使用者決定；[4] 題名 “…Coverage 3D Path Planning Method for Complex Areas”（DOI 10.1016/j.dt.2025.01.008）；PAPER_TEXT_ZH.md C7-a

**舊文**：

~~~~
Wang 等人[4]則以轉彎半徑 4000 m 之同質固定翼機隊驗證協同覆蓋規劃。含風時間成本[5]、
~~~~

**新文**：

~~~~
Wang 等人[4]則以同質固定翼機隊驗證複雜區域之協同覆蓋規劃。考慮風場之飛行時間成本[5]、
~~~~

### E10　C6-e、C7、決定 2（R_eff 為既有結果，引 [15]）

**證據**：PAPER_TEXT_ZH.md C6-e；使用者決定 2

**舊文**：

~~~~
(i) 以各構型含風有效轉彎半徑計算作業時間，
~~~~

**新文**：

~~~~
(i) 以各構型之風修正有效轉彎半徑[15]與考慮風場之地速計算作業時間，
~~~~

### E11　C2-a（模擬風於解鎖前讀回）

**證據**：runs/artifacts/HV4W55/wind_readback.json（15 列全數相符）

**舊文**：

~~~~
逐項讀回參數、比對任務項數，不符即停飛
~~~~

**新文**：

~~~~
逐項讀回參數與模擬風設定、比對任務項數，不符即停飛
~~~~

### E12　C1-f（h = 119.5 m）、C7-b（考慮風場之地速；直線與圓弧同式）

**證據**：NUMBERS.json survey_geometry；hv_dccpp/wind.py:278-291（ground_speed）、:370-421（直線段與弧段呼叫同一式）

**舊文**：

~~~~
取 $h$ = 120 m、$\alpha_H$ = 60°、$o_s$ = 0.30，得 $w$ = 138.6 m、$s$ = 97.0 m。時間成本採含風地速[5]與有效半徑 $R_{eff}$ 之 Dubins 路徑[6]：
~~~~

**新文**：

~~~~
取 $h$ = 119.5 m（規劃天花板）、$\alpha_H$ = 60°、$o_s$ = 0.30，得 $w$ = 138.0 m、$s$ = 96.6 m。時間成本以考慮風場之地速[5]與風修正有效轉彎半徑 $R_{eff}$ 之 Dubins 路徑[6]計算，直線段與圓弧段以同一地速式逐段積分：
~~~~

### E13　C6（式號順延：原式 (5) → (6)）

**證據**：本檔 2.3 節

**舊文**：

~~~~
$$\begin{aligned}&\mathrm{conflict}(a,b)\iff d_h<d_{thr}\ \wedge\ d_v<\Delta h_{sep}\ \wedge\ |t_a-t_b|<\Delta t+\sigma_t(t_a)+\sigma_t(t_b),\quad \sigma_t(t)=\min\{k_t\max(0,t-t_{sync}),\ \sigma_{\max}\}\\&\mathrm{site}(r_a)=\mathrm{site}(r_b)\ \wedge\ u_a\neq u_b\ \Rightarrow\ I(r_a)\cap I(r_b)=\varnothing\end{aligned} \tag{5}$$
~~~~

**新文**：

~~~~
$$\begin{aligned}&\mathrm{conflict}(a,b)\iff d_h<d_{thr}\ \wedge\ d_v<\Delta h_{sep}\ \wedge\ |t_a-t_b|<\Delta t+\sigma_t(t_a)+\sigma_t(t_b),\quad \sigma_t(t)=\min\{k_t\max(0,t-t_{sync}),\ \sigma_{\max}\}\\&\mathrm{site}(r_a)=\mathrm{site}(r_b)\ \wedge\ u_a\neq u_b\ \Rightarrow\ I(r_a)\cap I(r_b)=\varnothing\end{aligned} \tag{6}$$
~~~~

### E14　C6（式號順延：原式 (4) → (5)）

**證據**：本檔 2.3 節

**舊文**：

~~~~
$$\lambda_{floor}=\left\lceil\frac{T_{single}}{\kappa_{load}\,T_{use}-t_{in}-t_{RTB}}\right\rceil,\qquad T_{use}=\frac{3600\,E_{use}}{P_c\,k_{unc}} \tag{4}$$
~~~~

**新文**：

~~~~
$$\lambda_{floor}=\left\lceil\frac{T_{single}}{\kappa_{load}\,T_{use}-t_{in}-t_{RTB}}\right\rceil,\qquad T_{use}=\frac{3600\,E_{use}}{P_c\,k_{unc}} \tag{5}$$
~~~~

### E15　C7-c（完整風三角；空速型別）、C6-a（新增式 (4)）、C6-b＋決定 2（R_eff 為既有結果 [15]，本文之選擇為取最壞相位曲率）、C3-e（k 值與 W*）

**證據**：hv_dccpp/wind.py:278-306（ground_speed 即完整風三角）；hv_dccpp/platform.py:296-304（r_eff、w_star）；hv_dccpp/mission.py r_eff_wind_policy = "max_layer"；PAPERPLAN1/c6_reff/reff_numbers.json（binding_branch_at_W['5'] = R_plan）；k = ⌈2·200/96.59⌉ = 5、⌈2·120/96.59⌉ = 3

**舊文**：

~~~~
$W_\parallel$、$W_\perp$ 為風速之航跡切向與法向分量，$\delta$ 為弧段步長（≤5°）。當 $s<2R_{eff}$ 時，跳帶數依 span coefficient $k=\lceil 2R_{eff}/s\rceil$ 決定[7,8]（公式值：Pioneer 5、HERO 3）。各胞機數以決策級配額決定：
~~~~

**新文**：

~~~~
$W_\parallel$、$W_\perp$ 為風速沿航跡之切向與法向分量（完整風三角），$\delta$ 為弧段積分步長（≤5°）。有效轉彎半徑取定常風中轉彎地面軌跡（trochoid，次擺線[5,15]）之順風相位曲率半徑，並以規劃半徑為下限：

$$R_{eff,c}(W)=\max\Big\{R_{plan,c},\ R_{theo,c}\big(1+\tfrac{W}{V_{a,c}}\big)^2\Big\},\qquad R_{theo,c}=\frac{V_{a,c}^2}{g\tan\varphi_{max,c}},\qquad W^*_c=V_{a,c}\Big(\sqrt{\tfrac{R_{plan,c}}{R_{theo,c}}}-1\Big) \tag{4}$$

Word 線性格式：
```
R_(eff,c)(W)=max{R_(plan,c), R_(theo,c)(1+W/V_(a,c))^2},  R_(theo,c)=V_(a,c)^2/(g tan φ_(max,c)),  W_c^*=V_(a,c)(√(R_(plan,c)/R_(theo,c))−1)
```

此為既有結果[15]，非本文之貢獻；本文僅取此最壞相位曲率半徑作為地面座標之單一 Dubins 半徑，使條帶排序與成本表全程留在地面座標。$W$ 為作業高度帶內之最大層風速，$W^*$ 為規劃半徑之內建裕度可吸收之風速。當 $s<2R_{eff}$ 時，跳帶數依 span coefficient $k=\lceil 2R_{eff}/s\rceil$ 決定[7,8]（本文情境 W = 5 m/s 低於 $W^*$，$R_{eff}=R_{plan}$，k 為 Pioneer 5、HERO 3）。各胞機數以決策級配額決定：
~~~~

### E17　C6-d（掛符號、新增 W* 列）、C1-f（刈幅／間距）、N3（R_theo 幾何自洽值）、C6-f（HERO 26.07° 只加表註）

**證據**：TABLES_ZH.md 表 1；PAPERPLAN1/c6_reff/reff_numbers.json（W* 5.1675／5.0333、R_theo_formula 129.46／78.92 m）；PAPERPLAN1/c6_reff/ground_circle_bank.json（B_W5 bank_deg_on_R_eff 26.072°）；NUMBERS.json survey_geometry（137.9867／96.5907 m）

**舊文**：

~~~~
| 參數 | Pioneer（A 構型） | HERO（B 構型） |
|---|---|---|
| 巡航空速 | 21.5 m/s | 19.0 m/s |
| 轉彎半徑（理論／規劃，最大坡度） | 130／200 m（20°） | 75／120 m（25°） |
| 續航／可用航程（含 30% 保留） | 176 min／133–145 km | 136 min／79–86 km |
| 最低固定翼作業高度／空域上限 | 100 m／120 m AGL | 100 m／120 m AGL |
| 相機（兩型共用） | HFOV 60°、側疊 30% ⇒ 刈幅 138.6 m、間距 97.0 m | 同左 |

表註：規格為公開測試值與規劃值；電池能量、相機焦距（16 mm）與像素間距（3.9 μm）為假設值。
~~~~

**新文**：

~~~~
| 參數 | Pioneer（A 構型） | HERO（B 構型） |
|---|---|---|
| 巡航空速 $V_a$ | 21.5 m/s | 19.0 m/s |
| 轉彎半徑 $R_{theo}$／$R_{plan}$（最大傾角 $\varphi_{max}$） | 130／200 m（20°） | 75／120 m（25°） |
| 規劃裕度可吸收之風速 $W^*$（式 (4)） | 5.1675 m/s | 5.0333 m/s |
| 續航／可用航程（含 30% 保留） | 176 min／133–145 km | 136 min／79–86 km |
| 最低固定翼作業高度／空域上限 | 100 m／120 m AGL | 100 m／120 m AGL |
| 相機（兩型共用） | HFOV 60°、側疊 30% ⇒ 於 119.5 m 刈幅 137.99 m、間距 96.59 m | 同左 |

表註：規格為公開測試值與規劃值；電池能量、相機焦距（16 mm）與像素間距（3.9 μm）為假設值，續航與可用航程為估計值。$R_{theo}$ 為廠商值（幾何自洽值 129.46／78.92 m）；以表列值計，HERO 於 W = 5 m/s 之順風相位需傾角 26.07°，略大於 25°。
~~~~

### E18　決定 1（1.2434 km²）、C1-b（法規 121.92 m 與規劃天花板 119.5 m）、C2-a（規劃風與模擬風為獨立變因、解鎖前讀回）、K1（間距量尺）、N2（完工時間判準）

**證據**：NUMBERS.json survey_geometry（area_m2 1 243 447.5、flat 1 248 000）、conventions（separation_frame、makespan、coverage）；metrics.json *.altitude.per_uav.rate_hz（5.1–6.4 Hz）；HV4W55/wind_readback.json（15 列）；HV4W55/scenario.json

**舊文**：

~~~~
作業區為 1560 m × 800 m 平地矩形（1.248 km²），由系統自動切為 3 個子區；機隊為 1 架 Pioneer（A1）與 2 架 HERO（B2、B3），起飛坪相距 10 m，共用降落場坪 H；空域上限 120 m AGL 不申請豁免，最低固定翼作業高度 100 m，目標 GSD 0.05 m/px，規劃假設 5 m/s 西風。飛行採 ArduPlane 4.8.0-dev QuadPlane SITL（內建機體模型、5 倍速），依規劃起飛時刻逐機放行。覆蓋率以航跡加刈幅緩衝回算，間距以 2 s 網格內插航跡計算，兩者均為上界。
~~~~

**新文**：

~~~~
作業區為標稱 1560 m × 800 m 之平地矩形（WGS84 橢球面積 1.2434 km²），由系統自動切為 3 個子區；機隊為 1 架 Pioneer（A1）與 2 架 HERO（B2、B3），起飛坪相距 10 m，共用降落場坪 H；空域上限取法規四百呎（121.92 m）以下之 120 m AGL，不申請豁免，規劃天花板設為 119.5 m 以保留匯出離散化與高度追蹤之裕度（3.3 節）；最低固定翼作業高度 100 m，目標 GSD 0.05 m/px。規劃風與模擬風設為獨立變因（表3），主結果兩者均為 5 m/s、270°；模擬風於解鎖前讀回確認（三機 15 項），紊流為 0、不隨高度變化。飛行採 ArduPlane 4.8.0-dev QuadPlane SITL（內建機體模型、5 倍速），依規劃起飛時刻逐機放行。覆蓋率以航跡加刈幅緩衝回算，為上界；間距以飛控原生 5.1–6.4 Hz 紀錄於 WGS84 橢球座標求連續時間最小值；完工時間以最後一架下穿 2 m 判定。
~~~~

### E19　C1-c（高度句取代「貼齊上限」）、K1（100.43 m、垂直差 28.4 m）、C4-b＋決定 4（34 項之定義；91/96 到 float32 精度；85/96 逐位元）、K2

**證據**：NUMBERS.json trials.HV4W55.altitude（119.6221／119.6356／119.6752、t_above_120_s 0.0）、.separation（100.4256、vert_at_min_m 28.3942、margin 0.4256）、.params（strict_float32 91/96、strict_exact 85/96）；metrics.json HV4W55.timing.per_uav（dist_err 3.813／4.675／3.736%）；HV4W55/params_readback.csv（EXPECT_FALLBACK = ARMING_REQUIRE／SCHED_LOOP_RATE；FIRMWARE_OVERRIDE = Q_RTL_MODE；AIRSPEED_MAX 26.6 → 26.0）；NUMBERS.json corrections.K2_param_strict_comparison（6 筆 float32 往返）

**舊文**：

~~~~
結果如表2 與圖3。三機完成起降，高度貼齊 120 m 上限，航程較規劃多 2.2%–2.5%。最小水平間距 101.0 m 發生於 B2–B3（垂直差 29.0 m），雖無違規，但距門檻僅 1.0 m；同一計畫重飛為 104.2 m。參數讀回 102 筆中，含 3 筆韌體相容之刻意覆寫（Q_RTL_MODE）與 2 筆整數截斷（AIRSPEED_MAX）。
~~~~

**新文**：

~~~~
結果如表2（架次 HV4W55）。三機完成起降，全程最高 119.675 m AGL、高於 120 m 之時間 0 s；最小水平間距 100.43 m（B2–B3）僅餘 0.43 m（見限制 (2)）。參數每機比對 34 項：任務產生器匯出 32 項，另 2 項（`ARMING_REQUIRE`、`SCHED_LOOP_RATE`）為 SITL 平台設定而非韌體需求；「相符」指與韌體正規化後期望值之差 ≤ max(0.01, 0.1%)，正規化含 `Q_RTL_MODE` 刻意覆寫（3 機，韌體不接受其與 `RTL_AUTOLAND` 同時啟用）與 `AIRSPEED_MAX` 整數化（2 機）；對原始規劃值（96 項）則到 float32 精度為 91／96、逐位元為 85／96。
~~~~

### E20　C1／C2（主結果改為 HV4W55）、決定 1、K1、K2、C4-c、C2-c（表註第一句）、N2

**證據**：TABLES_ZH.md 表 2；NUMBERS.json trials.HV4W55（coverage、altitude.exported_max_rel_m 119.5526、makespan 1260.454 → 1281.906（+1.702%）、separation（planned 105.6992、n_paired_samples 7309、n_violations 0）、distance（+4.004%）、wind_test.signal_100m 9.9526／9.9314／9.9575）；metrics.json HV4W55.timing.per_uav（10.2936／7.9265／12.4406 → 10.686／8.297／12.9054 km）；HV4W55/{mission_upload.json, wind_readback.json, scenario.json}

**舊文**：

~~~~
**表2 規劃與 SITL 飛行結果對照（1 Pioneer + 2 HERO）**

| 項目 | 規劃 | SITL 飛行 |
|---|---|---|
| 起降完成 | 3 架 | 3／3 |
| 母區覆蓋率（1.248 km²） | 未覆蓋 0 | 100.0%（上界） |
| 全程最高高度 AGL（上限 120 m） | ≤120.05 m | 120.10／120.12／120.16 m |
| 航程 A1／B2／B3（km） | 10.29／7.93／12.44 | 10.52／8.13／12.71（+2.3%） |
| 完工時間 makespan（s） | 1260.4 | 1268.0 |
| 最小水平間距（m；門檻 100） | 105.5 | 101.0（垂直 29.0；違規 0/310） |
| 飛控參數讀回（34 項 × 3 機） | 匯出 32 項 × 3 機 | 102／102 相符 |
| 任務上傳（項數） | 82／53／84 | 讀回一致 |

表註：SITL 未設風，而規劃假設 5 m/s 西風，時間類比較可能含東西向誤差抵銷；makespan 以觸地且上鎖判定（以高度 <2 m 判定為 1257.0 s）；覆蓋率與間距為航跡回算上界。
~~~~

**新文**：

~~~~
**表2 規劃與 SITL 飛行結果對照（1 Pioneer + 2 HERO；架次 HV4W55）**

| 項目 | 規劃 | SITL 飛行 |
|---|---|---|
| 起降完成 | 3 架 | 3／3 |
| 母區覆蓋率（1.2434 km²） | 未覆蓋 0 | 99.97%（上界） |
| 全程最高高度 AGL（上限 120 m） | 匯出 ≤119.553 m | 119.622／119.636／119.675 m |
| 航程 A1／B2／B3（km） | 10.29／7.93／12.44 | 10.69／8.30／12.91（+4.0%） |
| 完工時間 makespan（s） | 1260.5 | 1281.9（+1.70%） |
| 最小水平間距（m；門檻 100） | 105.70 | 100.43（垂直 28.4；違規 0/7309） |
| 飛控參數讀回（34 項 × 3 機） | 匯出 32 項 × 3 機 | 102／102 相符（原始值：float32 精度 91／96，逐位元 85／96） |
| 任務上傳（項數） | 82／53／84 | 讀回一致 |

表註：規劃風與模擬風均為 5 m/s、270°（解鎖前讀回 15／15，風判準見表3）；百分比對完工時間判準（3.1 節）敏感，引用須連同本註；每機 32 項為本情境之匯出項數；`waiver_max_agl_m: 119.5` 為天花板設定而非豁免；平面近似之 1.248 km² 高估 0.37%。
~~~~

### E21　C1-d（離散化機制＋單變因對照，以 HV4W55R 為準）、K6（+0.0279 s）、C2-b（生效、忽略風之代價、買到的是準確度）、C2-h（側向分量；不跨風向比較）、C2-i＋K3（生成位置揭露）、K1（表3 間距）；圖3 改用 fig5_compliance

**證據**：NUMBERS.json ceiling_control（exported n_above_120 [0, 88]、flown max_agl [119.6752, 120.1742]、t_above_120_s [0.0, 354.925]、planned_makespan_cost_of_1195_s 0.027894、identical_files 6）；trials.*（makespan、separation.flown_exact_m／plan_minus_flown_m、altitude、wind_test.signal_100m／signal_30m／null_30m／ekf_wind）；crosstrack_wind_triangle.residual_mps（0.041／0.0061／0.0246）；corrections.K3_K4_spawn（err 5.0056／5.0607／5.1081 m、max_abs_spacing_delta 0.0218 m、benign cells ≤ 0.2146 m）；metrics.json *.windaxis.ew_half_sum（0.3864–0.4453）、*.timing.per_uav（A1 放行 612 → 656 s）；hv_dccpp/ardupilot.py:388-395、hv_dccpp/sim.py:58；c2_wind/evidence/r_eff_and_gust_limits.json（6.154 m/s）

**舊文**：

~~~~
> 【插入圖3：`fig3_results.png`，寬 17.0 cm，置中】
> 圖說：見附錄 C

#### 3.3 起降走廊判準對照

於 1 架 Pioneer 與 3 架 HERO、起飛坪相距 10 m 之情境，只切換走廊豁免判準（表3）。以座標判定時，相距 10 m 之不同場坪被視為同坪而豁免四維檢查，最小間距降至 5.0 m／19.9 m（B3–B4 起降段），規劃驗證卻仍通過、覆蓋率仍為 100%；改以場坪身分判定則無違規。此單一受控情境顯示，場坪語意與飛行安全直接相關，且覆蓋率僅為必要而非充分之驗收條件。
~~~~

**新文**：

~~~~
#### 3.3 規劃天花板與風場之對照

**規劃天花板。** 任務匯出將圓弧重新離散為多邊形、新頂點高度以線性內插，而等 AGL 曲面於局部東-北-天座標為拋物面，故迴轉頂點可高出天花板，且規劃驗證器（只檢查規劃航點、容差 1 m）無法察覺。HV4W55R 與 HV4W55 僅天花板不同（同風場、同匯出項數、參數檔逐位元相同）：120.0 m 時匯出 88 個航點高於 120 m、實飛最高 120.174 m、累計 354.93 s 高於 120 m，119.5 m 時皆為 0（表3、圖3(a)），代價為規劃完工時間 +0.0279 s，此係本作業區之性質，不宜外推。

**風場。** 表3 之運動學判準（東向減西向之地速 − 空速，理論 2W）與飛控 EKF3 風速（4.93–4.97 m/s）證明模擬風確實生效。HV4W05 與 HV4W00 飛同一任務檔、僅模擬風不同，完工時間增加 26.90 s；含風規劃之預測誤差較小（+1.70% 對 +2.43%），完工時間卻晚 47.13 s（44.0 s 為刻意延後放行），即買到的是預測準確度而非速度。風向 180° 之 HV4W55S 以北−南判準與側風地速虧損（與 $\sqrt{V_a^2-W^2}-V_a$ 相差 ≤ 0.041 m/s）驗證風三角之側向分量；兩風向為規劃器各自重解之計畫，不比較其完工時間。

**表3 規劃天花板與風場之對照架次（1 Pioneer + 2 HERO，同一情境）**

| 架次 | 天花板 (m) | 規劃風／模擬風 | 高於 120 m (s) | makespan 規劃→實飛 (s) | 實飛間距 (m) | 風判準 (m/s) |
|---|---|---|---|---|---|---|
| HV4SITL16 | 120.0 | 5/270°／無 | 330.14 | 1260.4→1255.9 | 100.685 | 東−西 −0.019 至 +0.005 |
| HV4W00 | 119.5 | 0／0 | 0 | 1205.5→1207.9 | 100.435 | 東−西 −0.009 至 +0.009 |
| HV4W05 | 119.5 | 0／5/270° | 0 | 1205.5→1234.8 | 100.291 | 東−西 +9.915 至 +9.969 |
| HV4W55 | 119.5 | 5/270°／5/270° | 0 | 1260.5→1281.9 | 100.426 | 東−西 +9.931 至 +9.958 |
| HV4W55R | 120.0 | 5/270°／5/270° | 354.93 | 1260.4→1282.2 | 100.579 | 東−西 +9.934 至 +9.960 |
| HV4W55S | 119.5 | 5/180°／5/180° | 0 | 966.2→996.9 | 266.177 | 北−南 +9.512 至 +9.853 |
| W ≥ 6.16 m/s | — | — | — | 規劃器判定 NO_GO，未出動 | — | — |

表註：(a) 風判準＝巡航段逐航向 90° 扇區之（地速 − 空速）中位數差，理論 2W = +10.0 m/s，並消去與航向無關之尺度偏移；180° 架次改取相對高度 30 m 以上樣本（其餘 100 m），其東−西為 −0.046 至 +0.014 m/s。(b) HV4W55S 三機於解鎖前沿地面偏離起飛站 5.0–5.1 m（剛體平移，間距變化 ≤ 0.022 m），係在明示且記錄之容差覆寫（2.0 → 6.0 m）下放行；位移方向鎖在機體縱軸，量值未能自模擬器原始碼導出；其餘七趟 ≤ 0.215 m。(c) 陣風取 1.3 倍風速、場址上限 8.0 m/s，W ≥ 6.16 m/s 即 NO_GO。(d) HV4SITL16 與 HV4SITL15（未列）為同計畫重飛（完工時間差 0.025 s），模擬未設風，與 119.5 m 各趟差兩個變因。(e) 規劃間距為 2 s 網格上界，較實飛高 4.20–5.27 m（HV4W55S 56.20 m）；違規皆 0；每格一趟。

> 【插入圖3：`fig5_compliance.png`，寬 17.0 cm，置中】
> 圖說：見附錄 C

#### 3.4 起降走廊判準對照

於 1 架 Pioneer 與 3 架 HERO、起飛坪相距 10 m 之情境，只切換走廊豁免判準（表4）。以座標判定時，相距 10 m 之不同場坪被視為同坪而豁免四維檢查，最小間距降至 5.0 m／19.9 m（B3–B4 起降段），規劃驗證卻仍通過、覆蓋率仍為 100%；改以場坪身分判定則無違規。此單一受控情境顯示，場坪語意與飛行安全直接相關，且覆蓋率僅為必要而非充分之驗收條件。
~~~~

### E22　表號順延（原表3 → 表4）

**證據**：本檔第三節

**舊文**：

~~~~
**表3 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**
~~~~

**新文**：

~~~~
**表4 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**
~~~~

### E23　決定 5（PAD8 未納管、第三方無從核對）、K1（表4 仍為舊量尺，須明講）；新增 3.5 節（C3-c、C3-f／K5：BEAMIDP1 只證可執行、完工時間不引用）

**證據**：`git ls-files runs/artifacts/PAD8_BROKEN_RAW runs/artifacts/PAD8_FIXED_RAW` = 0 筆（.gitignore:109）；PAPERPLAN1/prep_beamidp/tables.json（fleet seq_t 1696.88、bid_t 1115.73、hk_t 1060.15、fleet_delta_turn_time_pct −34.25）；各胞 n：Pioneer 6／7、HERO 13／7／6（同檔 table_md_en）；NUMBERS.json trials.BEAMIDP1（separation.flown_exact_m 180.7735、altitude.t_above_120_s 0.0）、corrections.K5_beamidp_makespan_confound（sim wind 0）；BEAMIDP1/mission_upload.json（263／165／175 送出＝讀回）

**舊文**：

~~~~
表註：回收站自動指派關閉；間距為 2 s 網格內插上界；單一情境、單次執行。
~~~~

**新文**：

~~~~
表註：回收站自動指派關閉；間距為 2 s 網格內插上界（既有報表程式之量尺，非 3.1 節之方法）；單一情境、單次執行；兩臂原始資料（`PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW`）未納入版本控制，第三方目前無從核對。

#### 3.5 跳帶排序之觸發情境

表2 情境之各胞條帶數（Pioneer 1、HERO 2）未超過 k，排序無作用（限制 (3)）。另設觸發情境（作業區改為三塊標稱 1600 × 1200 m，其餘同表2；各胞 n = 6–13 > k），以規劃器自身之考慮風場 Dubins 胞內成本評分：全機隊迴轉時間由順序掃描之 1696.88 s 降為 1115.73 s（−34.25%），距 Held–Karp 最佳解仍高 5.24%，均為規劃層模型值。交錯航線之可執行性另經 SITL 飛行確認（BEAMIDP1：三機完成起降、最小間距 180.77 m、高於 120 m 之時間 0 s；模擬未設風，其完工時間不引用）。
~~~~

### E24　結論（C1：刪「高度貼齊上限」；C2；C4-d；C7-a；K6 用 +0.0279 s）

**證據**：PAPER_TEXT_ZH.md「第四節結論」；NUMBERS.json ceiling_control、trials.HV4W55

**舊文**：

~~~~
HARVEST 將含風時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane；SITL 閉迴路飛行中，三機完成起降、高度貼齊上限、參數讀回全數相符，走廊判準對照則顯示覆蓋率不足以作為唯一驗收條件。
~~~~

**新文**：

~~~~
HARVEST 將考慮風場之飛行時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane。5 m/s 模擬風場之 SITL 閉迴路飛行中三機完成起降、最高高度低於 120 m、參數讀回 102／102 相符；單變因對照顯示規劃天花板降 0.5 m 即可消除 354.93 s 之超高飛行，代價 +0.0279 s；納入風場換得的是預測準確度而非較短之完工時間；走廊判準對照則顯示覆蓋率不足以作為唯一驗收條件。
~~~~

### E25　限制：C2-d（1）、K1（2）、C3-a（3，原句「少於 3」是錯的）、C5-b（4）、C6-c（5）、C2-e（6）、C1-e（7）、（8）

**證據**：NUMBERS.json trials.HV4W55.separation（margin 0.4256）、trials.HV4W05.separation（margin 0.2909）、trials.BEAMIDP1.altitude（margin 0.3031）；metrics.json *.altitude.per_uav（匯出超出 0.0136–0.1703、追蹤超出 0.0265–0.1475 m）；prep_beamidp/beamidp_table.CORRECTION.json（HERO 胞 42.35 s）；prep_baseline_control/out/stepP_final_table.json（efficiency 0.910–22.873%、coverage 0.5277／0.617、d_ceq 109.923／111.825 對 D_model 97.726 km）；c6_reff/reff_numbers.json（W* 5.1675／5.0333）；c2_wind/evidence/r_eff_and_gust_limits.json（6.154）；各趟 plan/*.param（RTL_RADIUS／WP_LOITER_RAD 八趟皆 200／120）；hv_dccpp/allocate.py:694-724

**舊文**：

~~~~
本研究之限制：(1) 僅以 SITL 內建機體模型驗證，且模擬未設風；(2) 覆蓋率與間距為航跡回算上界，主結果間距餘裕僅 1.0 m，且同型 HERO 之間無垂直隔離通道；(3) 驗證情境每胞條帶數少於 3，跳帶排序未被觸發；(4) 以最長單機航程衡量時，HARVEST 在封閉基準之 10 組情境中均較團隊既有之 DCCPP 實作長 0.9%–22.9%，本文不主張航程效率；(5) 各條件僅 1–2 趟、單一作業區形狀。後續將設定模擬風場重飛，並進行硬體上模擬（Simulation on Hardware）與轉彎半徑消融實驗。
~~~~

**新文**：

~~~~
本研究之限制：(1) 僅以 SITL 內建機體模型驗證；風場為單層、定常、無紊流；W ≥ 6.16 m/s 即 NO_GO，風場驗證上限為 5 m/s（情境預設值，非場址實測）。(2) 覆蓋率為上界；間距餘裕僅 0.426 m（八趟最緊 0.291 m），且最近接時刻垂直差 28.4 m 低於 50 m 門檻，水平間距為唯一防線，同型 HERO 間無垂直隔離通道；規劃間距為上界，不蘊含實飛 ≥ 100 m。(3) 跳帶排序之觸發條件為單胞條帶數 n > k = ⌈2R_eff/s⌉，而非「條帶數少於 3」；表2 情境 n = 1／2 未超過 k = 5／3，四種條帶序之迴轉成本逐項相同，此缺口源自切區結果；3.5 節之效果為模型值，順序掃描對照組無法實飛。(4) 最長單機航程於封閉基準 10 組情境均較團隊既有之 DCCPP 實作長 0.91%–22.87%，本文不主張航程效率；另 2 組 HARVEST 自判不可行而回報未覆蓋（覆蓋率 0.528、0.617），恰為基準航線超出配置器能量預算 12.5% 與 14.4% 者，其成因（切區後才計算每胞機數而不再細切）列為後續工作。(5) W = 5 m/s 低於 W* = 5.1675／5.0333 m/s，R_eff 退化為 R_plan（匯出之迴轉半徑逐趟為 200／120 m），其風修正僅在 W* 至 6.15 m/s 生效，本文未驗證。(6) 條帶方位不隨風向調整；270° 與 180° 架次為各自重解之不同計畫。(7) 圓弧匯出離散化缺陷（3.3 節）以降低天花板迴避而未修正：匯出偏差 +0.014 至 +0.170 m、飛控追蹤另 +0.027 至 +0.148 m，此裕度不可外推（最壞餘裕 0.303 m）。(8) 各條件僅 1 趟、單一作業區，SITL 為決定性模擬。後續將進行硬體上模擬、陣風與分層風對照及轉彎半徑消融實驗。
~~~~

### E26　決定 2（新增參考文獻 [15]）

**證據**：使用者提供之書目：Techy & Woolsey, JGCD 32(6), 2009, pp. 1736-1746, DOI 10.2514/1.44580（⚠ 本輪未連網，未以 Crossref 複核）

**舊文**：

~~~~
[14] ArduPilot Dev Team, "QuadPlane Overview", ArduPilot Plane Documentation, https://ardupilot.org/plane/docs/quadplane-overview.html, accessed September 14, 2026.
~~~~

**新文**：

~~~~
[14] ArduPilot Dev Team, "QuadPlane Overview", ArduPilot Plane Documentation, https://ardupilot.org/plane/docs/quadplane-overview.html, accessed September 14, 2026.
[15] Techy, L., and Woolsey, C. A., "Minimum-Time Path Planning for Unmanned Aerial Vehicles in Steady Uniform Winds", Journal of Guidance, Control, and Dynamics, Vol. 32, No. 6, pp 1736-1746, 2009.
~~~~

### E27　決定 2（先行文獻含 [15]）

**證據**：本檔參考文獻 [15]

**舊文**：

~~~~
| 各元件均有先行文獻 [5]–[13] |
~~~~

**新文**：

~~~~
| 各元件均有先行文獻 [5]–[13]、[15] |
~~~~

### E28　C2-g（原列「必須附 SITL 未設風表註」已過時：主結果現為設風架次）

**證據**：PAPER_TEXT_ZH.md C2-g；NUMBERS.json trials.HV4W05／HV4W55（+2.43% → +1.70%；1281.906 − 1234.780 = 47.126 s）；metrics.json 放行 612 → 656 s

**舊文**：

~~~~
| 「規劃與實飛時間只差數秒」 | 必須附「SITL 未設風」表註 | 東西向誤差會互相抵銷，時間吻合不代表排程正確 |
~~~~

**新文**：

~~~~
| 「規劃與實飛時間只差數秒」／「含風規劃較快」 | 「含風規劃之預測誤差較小（+1.70% 對 +2.43%），但完工時間反而長 47.13 s，其中 44.0 s 是刻意延後放行」 | 含風規劃買到的是預測準確度；百分比對 t0 判準敏感，須連同表2 表註引用 |
~~~~

### E29　C4-e＋K2（102/102 必須給定義；「嚴格 91/96」會被重算成 85/96）

**證據**：NUMBERS.json corrections.K2_param_strict_comparison

**舊文**：

~~~~
| 「參數讀回 68/68」 | 「102/102（34 項 × 3 機）」 | 舊數字只比了 17 個參數 |
~~~~

**新文**：

~~~~
| 「參數讀回 68/68」 | 「102/102（34 項 × 3 機）」 | 舊數字只比了 17 個參數 |
| 「參數讀回 102／102 相符」（不給定義）／「原始值嚴格比對 91／96」 | 「102／102 與韌體正規化後期望值相符；到 float32 精度之原始值比對為 91／96，逐位元為 85／96」 | 判準 max(0.01, 0.1%)；6 筆為 float32 往返表示誤差 |
~~~~

### E30　C3-b（門檻為 n > k）＋PAPER_TEXT 附錄 A 彙總之其餘新增列（C1、K1、K3–K6、K8、K9、C5、C6、C7、決定 1、決定 3）

**證據**：PAPER_TEXT_ZH.md「附錄 A 新增／修改彙總」；CORRECTIONS_K1_K9.md；c7_costfn（Pioneer 順風 26.5／逆風 16.5／側風 √(21.5²−5²) = 20.91 m/s）

**舊文**：

~~~~
| 「以 skip-5／skip-3 飛行」 | 「公式值 k = 5／3」 | 本次情境每胞條帶數 <3，未觸發 |
~~~~

**新文**：

~~~~
| 「以 skip-5／skip-3 飛行」 | 「公式值 k = 5／3；本情境每胞 n ≤ k，固定跳帶序列退化為順序掃描，未觸發」 | 真正門檻是 n > k，不是「條帶數少於 3」 |
| 「所飛之條帶序與順序來回掃描完全一致」 | 「本情境無可改變成本之排序自由度，四種序之成本逐項相同」 | 序不同（若干胞為 [1, 0]），相同的是成本 |
| 「Beam-IDP 與 Held–Karp 只差 0.01%」 | 「高出 5.24%，單胞最多 9.11%」 | 0.01% 是單機 n = 9 的探測 |
| 「BEAMIDP1 完工時間 −3.22%」 | 不列；若列須原句揭露「含風規劃、無風模擬」 | 與 HV4SITL15／16 同一缺陷（K5） |
| 「高度貼齊 120 m 上限」 | 「低於 120 m 作業上限（規劃天花板 119.5 m）」 | 120 m 天花板時實際超出；單變因對照量得 354.93 s |
| 「降天花板的代價只有 +0.02 s」 | 「+0.0279 s」或「1260.4261 → 1260.4540 s」 | +0.02 s 是四捨五入假影（K6） |
| 「最小水平間距 101.0 m」「重飛 104.2 m」 | 「100.43 m（HV4W55）」；HV4SITL15／16 為 100.69 m | 舊值用平面近似緯度換算＋2 s 網格上界 |
| 「與既有報表程式之值差 3.4–5.3 m」 | 「差 0.31–3.51 m（對 2 s 網格報表）或 0.34–0.90 m（對原生取樣平面框）」 | 3.4–5.3 m 不可重現（K1） |
| 「作業區 1.248 km²」 | 「1.2434 km²（WGS84 橢球）」 | 1.248 為平面近似，高估 0.37% |
| 「迎風時間成本」「逆風速度」「含風時間成本」 | 「考慮風場之飛行時間成本」 | 程式算完整風三角：Pioneer 順風 26.5／逆風 16.5／側風 20.9 m/s，三項皆有飛行佐證 |
| 「180° 比 270° 快」「側風預測較差」 | 不寫 | 兩者為規劃器各自重解之不同計畫 |
| 「R_eff 為本文提出」「R_eff 出自 [5]」「R_eff 保證任何相位傾角 ≤ φ_max」 | 「R_eff 為定常風中 trochoid 之既有結果[15]；本文之選擇是取最壞相位曲率」 | [5] 無 (1+W/V_a)²；HERO 在 W = 5 m/s 需 26.07° > 25° |
| 「擺線（cycloid）」 | 「trochoid（次擺線）」 | [5] 原文用 trochoid |
| 「Wang 等人以轉彎半徑 4000 m…」 | 不寫該數字 | 未經原文核對（決定 3） |
| 「表5 是同一把尺」 | 「HARVEST 的 0 來自其原生四維排程，基準沒有這個排程；代價為起飛佇列」 | 兩臂受檢物件不同（K9） |
| 「取自已納管的基準 CSV」 | 「該 CSV 目前未納管，`bench.py` 亦為已修改狀態」 | `git status` 為 `??` 與 ` M` |
| 「其餘 10 格只用到 17.0%–68.7%」／「量測工具校準相差 1.62%」 | 「23.2%–68.7%」／「誤差帶 −6.86% 至 +2.31%」 | 17.0%、1.62% 在產物中不存在 |
| 「180° 為純順風，把停放機體往北推 5 m」 | 「機體在地面自由偏航轉向來風，位移鎖在機體縱軸；5 m 是圓弧的弦，方位 43.6°–44.4°」 | 位移方位為東北（K3） |
| 「兩次獨立執行逐位元重現」 | 「生成位置逐位元重現」 | `spawn_readback.json`／`report.json` 兩檔不同（K4） |
| 「這個機體沒有電池遙測」 | 「SITL 記錄的是通用 12.6 V 模擬電池，不是規劃模型之電池，故能量餘裕為規劃模型值」 | `BAT` 訊息每趟每機都有（K8） |
~~~~

### E30b　表號順延（附錄 A 之「表3 即反例」→ 表4）

**證據**：本檔 3.4 節（原 3.3 節）

**舊文**：

~~~~
| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表3 即反例 |
~~~~

**新文**：

~~~~
| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表4 即反例 |
~~~~

### E31　附錄 B 出處全面改指向新架次（C1–C7、K1、K2、K7、決定 1、決定 5）

**證據**：各列自身所列之出處

**舊文**：

~~~~
| 1 Pioneer + 2 HERO、120 m、HFOV 60°、重疊 0.30、風 5 m/s／270°、GSD 0.05、h_minfw 100、起飛坪相距 10 m | `runs/artifacts/HV4SITL16/scenario.json` |
| 母區 1560 × 800 m、1.248 km²、自動切 3 子區（Z2_1／Z3_1／Z4_1） | 同上 `areas[0].polygon`；`plan/mission_summary.json` `uavs.*.strips` |
| 規劃航程 10.293／7.926／12.439 km；makespan 1260.43 s；三機降落 H | `HV4SITL16/plan/mission_summary.json` |
| 飛行航程 10.523／8.126／12.709 km；最高高度 120.10／120.12／120.16 m；makespan 1268.0（上鎖）／1257.0 s（<2 m） | `HV4SITL16/{A1,B2,B3}/flight.csv`，算法同 `HV4SITL15/hv_sitl_report.py`；彙整於 `figures_src/fig3_data.json` |
| 覆蓋率 100%（1,248,034.8 m²，刈幅 138.56 m） | 以 `hv_sitl_report.py:138-176` 算法重算 |
| 最小水平間距 101.0 m（B2–B3，垂直 29.0，t=440 s），違規 0/310；規劃 105.46 m | 重算；`HV4SITL16/validation.json` |
| 同規劃重飛 104.2 m | `HV4SITL15/separation_from_tracks.json` |
| 參數 102/102（匯出 93、韌體覆寫 3、常數備援 6；AIRSPEED_MAX 截斷 2） | `HV4SITL16/params_readback.csv` |
| 任務 82/53/84 | `HV4SITL16/mission_upload.json` |
| SIM_WIND 0 次；東西向地速差 0.008 m/s | `HV4SITL16/sitl_cmdlines.txt`；`flight.csv` 重算 |
| 表3：5.0/19.9 m、5/378；52.7/112.6 m、0/346 | `runs/artifacts/PAD8_BROKEN_RAW/`、`PAD8_FIXED_RAW/` 的 `separation_from_tracks.json`、`validation.json`、`coverage_from_tracks.json` |
| 最長單機航程比值 1.009–1.229 | `docs/hv_dccpp/封閉基準_2026-09-10.md:36-57` |
| 平台參數 | `docs/hv_dccpp/平台參數表.md`；`hv_dccpp/platform.py:435-478` |
| κ_load 0.85、k_unc 1.2075、d_sep 100/60、Δh_sep 50、k_t 0.05、Δt 2 | `hv_dccpp/mission.py:1035-1043, 1087-1092` |
| σ_max 45 s；走廊 60/30/90 s | `hv_dccpp/deconflict.py:75, 93`；`mission.py:1047-1048` |
| λ_floor 實作 | `hv_dccpp/allocate.py:697-723` |
| k = ⌈2R/s⌉ 實作 | `hv_dccpp/dubins.py:619-623` |
| 法規四百呎 | 全國法規資料庫 民用航空法 第99條之14（pcode=K0090001） |
| 參考文獻書目 | 12 個 DOI 以 Crossref 直查；[8][9][10][11][12] 內容以出版社 PDF 第一手核對 |
~~~~

**新文**：

~~~~
| 全部飛行數字（表2、表3、3.3 與 3.5 節、限制） | `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json`（自 24 個 .BIN 重算；明細在同目錄 `metrics.json`、`wind.json`、`corrections.json`、`spawn_verification.json`） |
| 主結果情境：1 Pioneer + 2 HERO、規劃天花板 119.5 m、HFOV 60°、重疊 0.30、規劃風與模擬風 5 m/s／270°、GSD 0.05、h_minfw 100、起飛坪相距 10 m | `runs/artifacts/HV4W55/scenario.json`、`wind_readback.json`（15／15） |
| 作業區 WGS84 1559.6 × 797.3 m、1.2434 km²（平面近似 1 248 000 m²，高估 0.37%）；自動切 3 子區 | `NUMBERS.json` `survey_geometry`；`HV4W55/plan/mission_summary.json` |
| 刈幅／間距：119.5 m 時 137.99／96.59 m；120.0 m 時 138.56／96.99 m | `NUMBERS.json` `survey_geometry.swath_and_spacing` |
| 規劃航程 10.2936／7.9265／12.4406 km；飛行 10.6860／8.2970／12.9054 km；makespan 1260.454 → 1281.906 s（下穿 2 m） | `update_paper/out/metrics.json` `HV4W55.timing`；`NUMBERS.json` `trials.HV4W55` |
| 最高高度 119.6221／119.6356／119.6752 m、高於 120 m 0.00 s；匯出最高 119.5526／119.5332／119.5331 m | `metrics.json` `HV4W55.altitude` |
| 覆蓋率 99.9666%（未覆蓋 415.2 m²，刈幅 137.99 m） | `NUMBERS.json` `trials.HV4W55.coverage` |
| 最小水平間距 100.4256 m（B2–B3，垂直 28.394 m），違規 0／7309；規劃 105.699 m | `NUMBERS.json` `trials.HV4W55.separation` |
| 參數 102/102（匯出 93、韌體覆寫 3、常數備援 6 即 `ARMING_REQUIRE`／`SCHED_LOOP_RATE`；`AIRSPEED_MAX` 整數化 2）；原始值比對到 float32 精度 91/96、逐位元 85/96；判準 max(0.01, 0.1%)；關閉整數救援則 100/102 | `HV4W55/params_readback.csv`；`NUMBERS.json` `corrections.K2_param_strict_comparison` |
| 任務 82/53/84（送出＝讀回） | `HV4W55/mission_upload.json` |
| 表3 與 3.3 節之風判準（東−西／北−南、EKF 風）與 30 m／100 m 門檻 | `NUMBERS.json` `trials.*.wind_test`；`update_paper/out/wind.json` `floors` |
| HV4SITL16 未設風：`sitl_cmdlines.txt` 無 `SIM_WIND`；巡航段東西向差 +0.005／−0.002／−0.019 m/s（HV4SITL15 為 −0.001／−0.018／+0.003） | `HV4SITL16/sitl_cmdlines.txt`；`NUMBERS.json` `corrections.K7_appendixB_276` |
| 天花板單變因對照：88 → 0 項、120.1742 → 119.6752 m、354.93 → 0.00 s、+0.0279 s | `NUMBERS.json` `ceiling_control`；`runs/artifacts/{HV4W55,HV4W55R}/plan/mission_summary.json` |
| 離散化成因與驗證器盲點 | `hv_dccpp/ardupilot.py:388-395`；`hv_dccpp/sim.py:58`（`_AGL_TOL_M = 1.0`） |
| 側風三角殘差 +0.041／+0.006／+0.025 m/s | `NUMBERS.json` `crosstrack_wind_triangle` |
| HV4W55S 生成位置 5.006／5.061／5.108 m、剛體平移 0.022 m | `HV4W55S/spawn_readback.json`；`update_paper/out/spawn_verification.json`；`NUMBERS.json` `corrections.K3_K4_spawn` |
| NO_GO 6.16 m/s（陣風 1.3 × W、上限 8.0 m/s；W = 7 回 9.1 > 8.0） | `hv_dccpp/integration/aeroplan_adapter.py:961`；`runs/artifacts/PAPERPLAN1/c2_wind/evidence/{planner_wind_sweep_2_4_6_7.json, r_eff_and_gust_limits.json}` |
| R_eff、W* 5.1675／5.0333、幾何自洽 129.46／78.92 m、HERO 傾角 26.07°；R_eff = R_plan（W = 5 m/s） | `hv_dccpp/platform.py:296-304`；`runs/artifacts/PAPERPLAN1/c6_reff/{reff_numbers.json, ground_circle_bank.json}`；各趟 `plan/*.param` 之 `RTL_RADIUS`／`WP_LOITER_RAD`（八趟皆 200／120） |
| 地速式（完整風三角） | `hv_dccpp/wind.py:278-306`；`hv_dccpp/ardupilot.py:653` |
| 表4：5.0/19.9 m、5/378；52.7/112.6 m、0/346 | `runs/artifacts/PAD8_BROKEN_RAW/`、`PAD8_FIXED_RAW/` 的 `separation_from_tracks.json`、`validation.json`、`coverage_from_tracks.json`（⚠ 被 `.gitignore:109` 排除，未納管） |
| 3.5 節：迴轉時間 1696.88 → 1115.73 s（−34.25%）、距離 −33.65%、能量 −32.53%、Held–Karp 1060.15 s；限制 (3)「成本逐項相同」（每個 HERO 胞 42.35 s／758.0 m） | `runs/artifacts/PAPERPLAN1/prep_beamidp/{tables.json, beamidp_table.json, beamidp_table.CORRECTION.json}`；BEAMIDP1 飛行數字 `NUMBERS.json` `trials.BEAMIDP1`、`BEAMIDP1/mission_upload.json` |
| 表5／限制 (4)：航程 0.91%–22.87%（平均 13.19%）、覆蓋 0.528／0.617、能量 112.5%／114.4%、133 km 時最緊 95.4%、誤差帶 −6.86% 至 +2.31%、24 組態、衝突 114／572／959／688／753、起飛佇列 330–6460 s、最後落地 2576.7–9851.3 s | `runs/artifacts/PAPERPLAN1/prep_baseline_control/out/{stepP_final_table.json, stepI_dplan_rule.json, stepC_kpi_scan_control.json}`；`NUMBERS.json` `corrections.K9_table_z_rulers`；`docs/hv_dccpp/封閉基準_2026-09-10.md:36-57` |
| 限制 (7)：匯出超出 +0.0136 至 +0.1703 m、追蹤超出 +0.0265 至 +0.1475 m、最壞餘裕 0.3031 m | `metrics.json` `*.altitude.per_uav`；`NUMBERS.json` `trials.BEAMIDP1.altitude` |
| 平台參數 | `docs/hv_dccpp/平台參數表.md`；`hv_dccpp/platform.py:435-478` |
| κ_load 0.85、k_unc 1.2075 = k_bm 1.15 × k_as 1.05、d_sep 100/60、Δh_sep 50、k_t 0.05、Δt 2 | `hv_dccpp/mission.py:1035-1043, 1087-1092`；`hv_dccpp/platform.py:951, 976-988` |
| σ_max 45 s；走廊 60/30/90 s | `hv_dccpp/deconflict.py:75, 93`；`mission.py:1047-1048` |
| λ_floor 實作 | `hv_dccpp/allocate.py:697-723` |
| k = ⌈2R/s⌉ 實作 | `hv_dccpp/dubins.py:619-623` |
| 法規四百呎 | 全國法規資料庫 民用航空法 第99條之14（pcode=K0090001） |
| 參考文獻書目 | 12 個 DOI 以 Crossref 直查；[8][9][10][11][12] 內容以出版社 PDF 第一手核對；⚠ [15] 未以 Crossref 複核（本輪未連網），[4] 之內文數字未核對故已刪 |
~~~~

### E32　決定 2（[15] 之 DOI）

**證據**：使用者提供 DOI 10.2514/1.44580

**舊文**：

~~~~
[2] 10.1023/A:1008958800904　[3] 10.1007/s10846-016-0461-x　[4] 10.1016/j.dt.2025.01.008　[5] 10.3390/s18072132　[6] 10.2307/2372560　[7] 10.1016/j.biosystemseng.2008.06.008　[8] 10.3390/aerospace9090521　[9] 10.3390/math14030416　[10] 10.1007/s10846-016-0458-5　[11] 10.3390/drones7110664　[12] 10.3390/drones9090658　[13] 10.3390/drones9040287
~~~~

**新文**：

~~~~
[2] 10.1023/A:1008958800904　[3] 10.1007/s10846-016-0461-x　[4] 10.1016/j.dt.2025.01.008　[5] 10.3390/s18072132　[6] 10.2307/2372560　[7] 10.1016/j.biosystemseng.2008.06.008　[8] 10.3390/aerospace9090521　[9] 10.3390/math14030416　[10] 10.1007/s10846-016-0458-5　[11] 10.3390/drones7110664　[12] 10.3390/drones9090658　[13] 10.3390/drones9040287　[15] 10.2514/1.44580（未複核）
~~~~

### E33　圖說：舊圖3（HV4SITL16 內嵌舊值）改為 fig5_compliance（圖3）；fig4_wind 為備用圖4；C1、C2

**證據**：docs/hv_dccpp/AASRC2026_abstract/fig5_compliance.png、fig4_wind.png（本輪目視核對圖面內容）；figures_src/fig5_data.json、fig4_data.json

**舊文**：

~~~~
圖說置於圖下緣置中，11 點，各一行：

- 圖1 HARVEST 系統架構與本文 SITL 驗證路徑（粗箭頭）
- 圖2 HARVEST 規劃演算法流程（灰底點劃框為不可行或未覆蓋出口，虛線為再分配回圈）
- 圖3 SITL 飛行結果：(a) 規劃與飛行航跡，(b) 高度剖面，(c) 同時在空之機間間距
~~~~

**新文**：

~~~~
圖說置於圖下緣置中，11 點：

- 圖1 HARVEST 系統架構與本文 SITL 驗證路徑（粗箭頭）
- 圖2 HARVEST 規劃演算法流程（灰底點劃框為不可行或未覆蓋出口，虛線為再分配回圈）
- 圖3（檔案 `fig5_compliance.png`）高度合規與實飛間距：(a) 各趟匯出與實飛最高相對高度（HV4SITL15／16 為 120.0 m 天花板且模擬未設風之歷史對照；單變因對照 HV4W55R 見表3），(b) 規劃最小水平間距（2 s 網格上界）與實飛值（WGS84 橢球連續時間最小值；× 為既有平面近似值）
- 備用圖4（檔案 `fig4_wind.png`，本版未排入）風場對照：(a) 巡航段逐航向之（地速 − 空速）中位數，東−西差約為 2W；(b) 規劃風／模擬風組合下之機隊完工時間
~~~~

### E34　附錄 D 第 2 項（證據出處）更新：八趟已納管；PAD8、.BIN、基準 CSV、本資料夾仍未納管

**證據**：`git ls-files`：runs/artifacts/{HV4SITL15,HV4SITL16,HV4W00,HV4W05,HV4W55,HV4W55R,HV4W55S,BEAMIDP1} 各有 27–39 個納管檔；PAD8_BROKEN_RAW、PAD8_FIXED_RAW、docs/hv_dccpp/AASRC2026_abstract/、research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv 皆 0 筆；update_paper/COMMIT_PLAN_ADDENDUM.md §2

**舊文**：

~~~~
2. **證據出處不乾淨**：HV4SITL16 與 PAD8 兩臂都是在「HEAD 5e0295f + 未提交修改」下飛的。HV4SITL16 未納入 git；PAD8_* 被 `.gitignore:109` 排除，而且兩臂的建置身分只能靠 `docs/hv_dccpp/VENDOR.md` 的敘述證明。依專案紀律，這類數字不應進正式論文 ⇒ 建議在乾淨 commit 上重飛這三趟並納管，數字若變動再更新表2、表3、圖3。
~~~~

**新文**：

~~~~
2. **證據出處（v2026-09-21 更新）**：八趟飛行證據（HV4SITL15／16、HV4W00／05／55／55R／55S、BEAMIDP1）已於 95a337b 納管，重算程式與文字包於 3442fcb 納管。仍未乾淨者：(a) 各趟 `.BIN` 未納管，故「.BIN → `out/*.json`」一段只有持有 log 者能重跑；(b) 表4 兩臂 `PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW` 仍被 `.gitignore:109` 排除（表4 已加註第三方無從核對）；(c) 附錄 E 表5 航程列之 `project_rows_5e0295f5_dirty.csv` 未納管、`bench.py` 為已修改狀態；(d) **本資料夾（含本檔與 fig4／fig5）尚未納管**，檔頭版本戳所指 commit 不含本檔；納管後請更新版本戳。
~~~~

### E35　附錄 D 第 4 項（舊「只比門檻多 1.0 m」已過時）＋新增待辦（圖1–3 圖面舊字、[15] 核對、已知缺陷）＋附錄 E（C1–C7 對照、備用 3.6 節／表5＝C5 之主要回答）

**證據**：NUMBERS.json trials.*.separation；fig1_architecture.png（「含風 Dubins 時間成本」「兩者皆為上界」）、fig2_flowchart.png（「含風時間成本」）、fig3_results.png（101.0 m、0/310）；update_paper/README.md §四；PAPER_TEXT_ZH.md C5、C7-f；TABLES_ZH.md 表 Z；表5 數字同 E31 附錄 B 之表5 列

**舊文**：

~~~~
4. 最小間距只比門檻多 1.0 m，而且是 HERO–HERO 同高度帶。審稿人可能會追問，建議準備設風重飛與加密遙測（≥2 Hz）的補充數據。
~~~~

**新文**：

~~~~
4. 最小間距只比門檻多 0.43 m（主結果；本文八趟中最緊 0.29 m），且是 HERO–HERO 同高度帶，最近接時刻垂直差 28.4 m。設風重飛與原生取樣均已完成，量尺更準後餘裕反而變小；審稿人可能追問，答法：此為情境 `gui_legal_120_1a2b` 本身之性質，天花板改動既未造成也未修好（HV4W55R 為 100.579 m）。
5. **圖面上的舊字（本 label 不能改圖）**：`fig3_results.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 m），本版已自排版移除，若要恢復三聯圖須以 HV4W55 重產；`fig1_architecture.png` 仍寫「含風 Dubins 時間成本」與「覆蓋率與機間間距（兩者皆為上界）」、`fig2_flowchart.png` 仍寫「含風時間成本」，與正文 C7 用詞（考慮風場之飛行時間成本）及 3.1 節之間距量尺不一致，重產時請改；`fig5_compliance.png` 之 (a) 未含 HV4W55R，若重產建議加入。
6. 參考文獻 [15]（Techy & Woolsey 2009）之書目欄位未經 Crossref 逐欄核對；本文依使用者決定將 R_eff 歸於 [15]，投稿前請核對原文確有順風相位曲率半徑之結果，否則改寫為「由 trochoid[5,15] 之運動學直接可得」。
7. `reduce_waypoints()` 之圓弧離散化缺陷未修（本文以 119.5 m 天花板迴避，見限制 (7)）。另 GUI 匯出路徑 `mission/vtol_mission_exporter.py:135` 之 `DO_CHANGE_SPEED` 仍送地速（param1 = 1），與本文 SITL 路徑（`hv_dccpp/ardupilot.py:653`，空速）不同；論文數據不受影響，但被問及「匯出至 ArduPilot」時須知。

## 附錄 E　審閱意見 C1–C7 對照與備用內容（給使用者與指導老師，不放進 Word）

| 意見 | 本版回應位置 | 主要證據 |
|---|---|---|
| C1 120 m 高度主張與數據不符 | 摘要、前言、2.3（h = 119.5 m）、3.1、3.2、3.3＋表3＋圖3(a)、限制 (7) | `NUMBERS.json` `ceiling_control`；HV4W55 對 HV4W55R |
| C2 含風規劃卻在無風 SITL 驗證 | 摘要、2.1、3.1、3.3＋表3、表2 表註、限制 (1)(6)；備用圖4 | `update_paper/out/wind.json`；HV4W00／05／55／55S |
| C3 Beam-IDP 跳帶從未觸發 | 2.3（k 值）、3.5、限制 (3)、附錄 A | `prep_beamidp/tables.json`；BEAMIDP1 |
| C4 102 項參數之定義 | 摘要、3.2、表2 參數列與表註 | `HV4W55/params_readback.csv`；K2 |
| C5 DCCPP 基準較短 | 限制 (4)；完整回答為下方備用 3.6 節＋表5 | `prep_baseline_control/out/stepP_final_table.json`；K9 |
| C6 R_eff 未定義 | 前言貢獻 (i)、2.3 式 (4) 與說明、表1、限制 (5)、參考文獻 [15] | `c6_reff/reff_numbers.json`；`hv_dccpp/platform.py:296-304` |
| C7「迎風」與實際風三角 | 摘要、前言、2.3 式 (3) 之說明、附錄 A | `hv_dccpp/wind.py:278-306` |

### 備用 3.6 節（C5 之完整回答；Word 實測有餘時接在 3.5 節之後，須連同表5 與表註整段排入）

#### 3.6 與既有 DCCPP 實作之對照

以同一套檢查程式與門檻比較 HARVEST 與團隊既有之 DCCPP 實作（封閉基準；表5）。HARVEST 之最長單機航程較長，換得的是能量可行性與四維排程，代價為起飛佇列。

**表5 HARVEST 與 DCCPP 基準之能力／KPI 對照（封閉基準）**

| 指標 | DCCPP 基準 | HARVEST |
|---|---|---|
| 最長單機航程（10 組） | 較短 | 長 0.91%–22.87%（平均 13.19%） |
| 覆蓋率（12 組） | 12／12 組為 1.000 | 10／12 組為 1.000；2 組自判不可行（0.528、0.617） |
| 能量可行性（12 組；預算 $D_{model}$ = 97.726 km） | 2／12 組超標（112.5%、114.4%），且規劃器不自報 | 0／12 組超標；自判不可行者恰為該 2 組 |
| 四維間隔／起降走廊衝突（10 組） | 114／572 件（重抬升航跡；加上其自帶避撞後仍為 114／572） | 0／0 件（原生四維排程；若重抬升則為 959／688） |
| 代價：起飛佇列／最後落地 | 0 s（全機同時起飛） | 330–6460 s／2576.7–9851.3 s |

表註：衝突數以同一檢查程式與門檻計（$d_{sep}$ = 100 m、$\Delta h_{sep}$ = 50 m；降落 90 s、起飛 60 s，單一場站），但兩臂之受檢物件不同：基準無四維排程，只能在重抬升航跡（全機 t = 0）上受檢，HARVEST 則在其原生排程上受檢；HARVEST 之 0 來自其自身之四維排程，代價即末列之起飛佇列，本文不主張兩者為同一把尺。基準有被給予其協調模式（24 組態之任何指標皆零影響）與自帶之事後避撞模組（偵測並處理 753 件後讀數不變），未被給予能量模型，因其程式碼本無此模型。若改以表1 之可用航程 133 km 為預算，基準 12／12 組皆通過（最緊者 95.4%）；能量估計器對 HARVEST 自身計畫之誤差帶為 −6.86% 至 +2.31%。航程列之底層資料未納入版本控制，且計算程式 `bench.py` 為已修改狀態，第三方目前無法獨立重現。

完整之表 2b（七列風場矩陣）、表 2c（天花板對照全欄）、表 X（逐胞）、表 Y、表 Z（含全文圖說）見 `runs/artifacts/PAPERPLAN1/update_paper/TABLES_ZH.md`；本檔正文表3 與上方表5 為其壓縮版。
~~~~

> 註：E06 新文中的 `__PAGES_*__` 於寫檔時以第一節的估計頁數取代（見寫出之檔案 0.4 節）。

## 三、數字不一致（以 NUMBERS.json 或原始證據為準）

| 項目 | 文字包原寫 | 本版採用與依據 |
|---|---|---|
| HV4W55S 之「東−西」歸零軸區間 | PAPER_TEXT_ZH.md §0.4 與 README.md 一.5 寫「−0.032 ~ +0.014」，但同檔 C2-h 逐機列出 −0.032／−0.046／+0.014 | NUMBERS.json `trials.HV4W55S.wind_test.null_30m` = −0.0323／−0.046／+0.0139（該趟所用之 30 m 門檻）⇒ 表3 表註 (a) 寫 **−0.046 至 +0.014 m/s** |
| 30 m 門檻下 270° 各趟之「東−西」 | TABLES_ZH.md 表 2b 表註 (a) 寫「+9.966…+9.977」 | NUMBERS.json `trials.{HV4W05,HV4W55,HV4W55R}.wind_test.signal_30m` 實為 9.9488–9.9803 ⇒ 應為 +9.949 至 +9.980；本版**未寫入論文**（表註 (a) 已刪該括號），僅記錄 |
| 表2 違規取樣分母 | TABLES_ZH.md 表 2 寫「違規 0／4 714 對取樣」＝B2–B3 單一機對之取樣數（metrics.json `HV4W55.separation.frames.ecef.pairs.B2-B3.n_samples`） | NUMBERS.json `trials.HV4W55.separation.n_paired_samples` = **7309**（A1–B3 2595 ＋ B2–B3 4714）；違規 0 為全機隊之數，故表2 用 0/7309 |
| BEAMIDP1 違規取樣分母 | PAPER_TEXT_ZH.md C3-f／TABLES_ZH.md 表 X 腳註寫「違規 0／22 329 對取樣」＝A1–B2 單一機對 | NUMBERS.json `trials.BEAMIDP1.separation.n_paired_samples` = 63235；3.5 節未印分母，僅記錄 |
| k_bm、k_as 之名稱 | PAPER_TEXT_ZH.md C7-g 寫「分別為電池老化與空速裕度係數」 | `hv_dccpp/platform.py:951` 註明為「未驗證保守係數」，`:976-979` 在 `battery_monitor_validated`／`airspeed_sensor_validated` 為真時強制 1.0 ⇒ 應為「電池監測器與空速感測器未驗證之保守係數」。C7-g 之句因版面**未寫入正文**；附錄 B 出處列已照程式寫 |
| R_eff 可驗證窗口下緣 | PAPER_TEXT_ZH.md C6-c 與 README 二.1 寫「W ≳ 5.2 m/s」「5.2–6.15 m/s」 | 5.2 只對 Pioneer 成立（W* = 5.1675）；HERO 之 W* = 5.0333（`c6_reff/reff_numbers.json`；`c2_wind/evidence/r_eff_and_gust_limits.json` 以格點記為 5.1）⇒ 限制 (5) 寫 **「W* 至 6.15 m/s」** |
| 含風規劃晚 47.13 s 之拆解 | PAPER_TEXT_ZH.md C2-b 寫「44.0 s 來自延後放行，只有 3.2 s 來自飛得較久」 | 47.126 − 44.0 = 3.13 s，且 3.2 不在 NUMBERS.json ⇒ 論文只寫 44.0 s（metrics.json 放行 612 → 656 s），**刪去 3.2 s** |
| 天花板代價之位數 | PAPER_TEXT_ZH.md 第四節結論寫「+0.028 s」 | K6 定稿措辭為 +0.0279 s（NUMBERS.json `ceiling_control.planned_makespan_cost_of_1195_s` = 0.027894）⇒ 全文一律 **+0.0279 s** |
| R_eff 之式號 | PAPER_TEXT_ZH.md C6-a 說插在「原式 (3) 之前」（會成為式 (3)），但 C7-b 與 TABLES_ZH.md 表 1 表註都寫「式 (4)」 | 本版把 R_eff 放在地速式 (3) 之後成為 **式 (4)**，與 C7-b、表 1 一致；原 (4)(5) 順延為 (5)(6) |
| 固定 skip-k 合計與完工時間推估 | TABLES_ZH.md 表 X 寫 1 196.47 s、推估 2 532.6 s | `prep_beamidp/tables.json` 為 skip_t 1196.48、makespan_sequential_estimate_s 2532.5 ⇒ 兩者**皆未寫入論文**，僅記錄 |
| 表2 參數列用詞 | TABLES_ZH.md 表 2 寫「到 float32 精度之原始值**嚴格**比對為 91／96」 | K2 定稿措辭不含「嚴格」⇒ 表2 寫「原始值：float32 精度 91／96，逐位元 85／96」 |
| 版本戳所指 commit 不含本檔 | 決定 6 之版本戳「對應 commit 3442fcb」 | `git ls-files docs/hv_dccpp/AASRC2026_abstract` = 0 筆（整個資料夾未納管）⇒ 版本戳照寫，並於附錄 D 第 2(d) 項載明「commit 不含本檔，納管後請更新」 |
| 參考文獻 [15] 之內容未核對 | 決定 2 要求寫明 R_eff 為 [15] 之既有結果 | 本輪未連網；[15] 之書目與「順風相位曲率半徑 R_theo(1+W/V_a)²」是否出現在其原文均未核對 ⇒ 照決定寫入，列入附錄 D 第 6 項 |

## 四、刻意**未改**之處與理由

| 位置 | 理由 |
|---|---|
| 題目（主推與兩個備選）、作者、服務單位、關鍵詞 | 七項批註皆未觸及 |
| 0.1（除式號）、0.2 排版約定 | 與內容無關；只把式號 (1)…(5) 改為 (1)…(6) |
| 2.1 第一、三句；2.2 全段 | 無批註觸及；2.1 只在讀回項目加「與模擬風設定」（C2） |
| 式 (1)、(2)、原式 (4)(5)（現 (5)(6)）之內容與其 Word 線性格式；常數段（d_thr、Δh_sep、Δt、k_t、σ_max、I(r)）；k_unc 之句 | 無批註觸及；只改式號。C7-g 建議補 k_unc 之拆解，因版面未寫入正文（出處在附錄 B） |
| 3.4 節（原 3.3）正文與表4 之數值（5.0／19.9 m、5／378；52.7／112.6 m、0／346） | PAD8 兩臂未納管、本輪未重算（update_paper 不涵蓋 PAD8）；只改表號並在表註加「沿用舊量尺」「未納管、第三方無從核對」（決定 5），數值原樣保留 |
| 誌謝 | 待使用者填寫 |
| 參考文獻 [1]–[14] 之書目 | [4] 只刪正文中之「4000 m」（決定 3），書目本身未動 |
| 附錄 A 其餘既有列 | 仍正確；「首次…」一列僅補 [15] |
| 附錄 D 第 1、3 項 | 填表與 HV_TRIAL 警語仍有效 |
| `AASRC2026-投稿格式_加長型摘要_.doc`、全部圖檔與 `figures_src/` | 任務規定不碰 .doc；圖檔不在本 label 可改範圍 |
| 圖面上的舊字：`fig1_architecture.png`「含風 Dubins 時間成本」「（兩者皆為上界）」、`fig2_flowchart.png`「含風時間成本」、`fig3_results.png` 之 101.0 m 等 | 不能改圖；圖3 改用 fig5_compliance，圖1／圖2 之用詞問題列入附錄 D 第 5 項待重產 |

## 五、文字包中**未放進印刷正文**的內容（4 頁限制；仍可在 TABLES_ZH.md／PAPER_TEXT_ZH.md 或本檔附錄找到）

- 表 2b 全欄（機隊航程、規劃間距、覆蓋率；逐機三元組；HV4SITL15 列）與表 2c 全欄 ⇒ 正文表3 為壓縮版（區間取代逐機值），全欄見 TABLES_ZH.md（附錄 E 指路）
- 表 X 逐胞表、表 Y ⇒ 3.5 節只寫機隊合計與百分比
- 表 Z（C5 之主要回答）⇒ 壓縮為表5，放在附錄 E「備用 3.6 節」，Word 實測有餘才排入；正文以限制 (4) 回答 C5。表 Z 第 6、8 列與第 1 列絕對 km 範圍未收
- 圖4（fig4_wind.png）⇒ 備用，未排入（其數字全在表3）
- 放行至落地（gate-to-gate）預測誤差 +2.90 ~ +5.94 % 對 +3.11 ~ +4.90 % ⇒ 正文只寫「不比較兩風向之完工時間」
- 生成位移機制細節（SIM_QuadPlane 地面分支、偏航 127°→174.4°、方位 43.6°–44.4°）⇒ 表3 表註 (b) 保留結論句與「量值未能導出」
- BEAMIDP1 之完工時間（K5）、順序掃描完工時間推估（2 532.6 s）、迴轉距離 −33.65%／能量 −32.53%、固定 skip-k 合計 ⇒ 3.5 節只寫迴轉時間 −34.25% 與距 Held–Karp 5.24%
- C4-b「關閉整數救援則 100／102」⇒ 只在附錄 B
- C7-c 之航向式 ψ = θ − arcsin(W⊥/V_a)、地速上限綁住時逐條帶重選 V_a、`DO_CHANGE_SPEED` 為空速 ⇒ 正文只寫「完整風三角」，出處在附錄 B
- C6-b 中「空速座標生成再轉換至地面會成移動目標問題」之理由與 W = 6／8 m/s 之 R_eff 數值 ⇒ 正文只寫「取最壞相位曲率作為地面座標之單一 Dubins 半徑」
- C7-d 風三角數值（26.5／16.5／20.9 m/s）⇒ 附錄 A；C7-f GUI 匯出送地速 ⇒ 附錄 D 第 7 項
- C1-b「三機總航程 +2.4 m、規劃間距 105.464 → 105.699 m」「3 000 × 3 000 m 幾何上 −21.6 %」⇒ 正文改寫為「此係本作業區之性質，不宜外推」；+2.4 m 與規劃間距變化均未收
- C1-c「距法規 121.92 m 餘裕 2.245 m」與逐機最高高度文字 ⇒ 逐機值在表2
- C2-b 之 EKF3 風向 270.0°–270.5°、180° 之 180.1°–180.4°、東−西 +9.915 至 +9.969 之文字版 ⇒ 表3 帶區間，文字只引 EKF3 風速
- 限制 (6) 之「未採用[5]之垂直於風掃描」、限制 (1) 之「陣風取 1.3 倍」細節 ⇒ 分別刪去與移至表3 表註 (c)
- K8 電池遙測 ⇒ 只在附錄 A（論文正文原本就沒有這句）

## 六、舊主張殘留檢查（寫檔後逐字 grep；「印刷區」＝第 1、2 節，會進 Word）

預期：印刷區內之殘留只能是 (i) 歷史對照架次 HV4SITL16 之列與表註、(ii) 對 BEAMIDP1／HV4SITL16「未設風」之揭露、(iii) 限制 (3) 中以否定句出現的「少於 3」、(iv) 表2 表註中「平面近似之 1.248 km² 高估 0.37%」之說明；其餘出現處均在附錄 A（禁用欄）、附錄 B（出處）或附錄 D（待辦）。

### `貼齊`（舊主張「高度貼齊 120 m 上限」）：全檔 2 處，其中印刷區（第 1、2 節）0 處
- L42（第 0 節排版指示（不印））：…- **所有段落（含表格、圖說、參考文獻）取消「文件格線貼齊」**：段落 → 縮排與行距 → 取消勾選「文件格線被設定時，貼齊格線」。格式檔…
- L311（附錄）：…¦ 「高度貼齊 120 m 上限」 ¦ 「低於 120 m 作業上限（規劃天花板 119.5 …

### `迎風`（C7 舊用詞）：全檔 2 處，其中印刷區（第 1、2 節）0 處
- L316（附錄）：…¦ 「迎風時間成本」「逆風速度」「含風時間成本」 ¦ 「考慮風場之飛行時間成本」 ¦ 程式…
- L397（附錄）：…¦ C7「迎風」與實際風三角 ¦ 摘要、前言、2.3 式 (3) 之說明、附錄 A ¦ `hv…

### `含風時間成本`（C7 舊用詞（成本名稱））：全檔 2 處，其中印刷區（第 1、2 節）0 處
- L316（附錄）：…¦ 「迎風時間成本」「逆風速度」「含風時間成本」 ¦ 「考慮風場之飛行時間成本」 ¦ 程式算完整風三角：Pioneer 順風 …
- L383（附錄）：…率與機間間距（兩者皆為上界）」、`fig2_flowchart.png` 仍寫「含風時間成本」，與正文 C7 用詞（考慮風場之飛行時間成本）及 3.1 節之間距量尺不一致，…

### `含風有效`（C6／C7 舊用詞）：全檔 0 處，其中印刷區（第 1、2 節）0 處

### `101.0 m`（K1 舊間距）：全檔 3 處，其中印刷區（第 1、2 節）0 處
- L17（第 0 節排版指示（不印））：…⚠ **本版不放進 Word**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更…
- L313（附錄）：…¦ 「最小水平間距 101.0 m」「重飛 104.2 m」 ¦ 「100.43 m（HV4W55）」；HV4SI…
- L383（附錄）：…）**：`fig3_results.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 m），本版已自排版移除，…

### `104.2`（K1 舊重飛間距）：全檔 1 處，其中印刷區（第 1、2 節）0 處
- L313（附錄）：…¦ 「最小水平間距 101.0 m」「重飛 104.2 m」 ¦ 「100.43 m（HV4W55）」；HV4SITL15／16 為 …

### `1.248 km`（決定 1 舊分母）：全檔 2 處，其中印刷區（第 1、2 節）1 處
- L218（印刷區）：…iver_max_agl_m: 119.5` 為天花板設定而非豁免；平面近似之 1.248 km² 高估 0.37%。…
- L315（附錄）：…¦ 「作業區 1.248 km²」 ¦ 「1.2434 km²（WGS84 橢球）」 ¦ 1.248 為平面近…

### `少於 3`（C3 錯誤門檻）：全檔 2 處，其中印刷區（第 1、2 節）1 處
- L264（印刷區）：…帶排序之觸發條件為單胞條帶數 n > k = ⌈2R_eff/s⌉，而非「條帶數少於 3」；表2 情境 n = 1／2 未超過 k = 5／3，四種條帶序之迴轉成本逐項…
- L307（附錄）：…，固定跳帶序列退化為順序掃描，未觸發」 ¦ 真正門檻是 n > k，不是「條帶數少於 3」 ¦…

### `<3`（C3 錯誤門檻（附錄 A 舊寫法））：全檔 0 處，其中印刷區（第 1、2 節）0 處

### `138.6`（C1-f 舊刈幅）：全檔 0 處，其中印刷區（第 1、2 節）0 處

### `97.0 m`（C1-f 舊間距）：全檔 0 處，其中印刷區（第 1、2 節）0 處

### `HV4SITL16`（舊主結果架次）：全檔 7 處，其中印刷區（第 1、2 節）2 處
- L17（第 0 節排版指示（不印））：…sults.png`（.svg） ¦ ⚠ **本版不放進 Word**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120…
- L64（第 0 節排版指示（不印））：…5. 表3 刪 HV4SITL16 與「W ≥ 6.16 m/s」兩列（表註 (c)(d) 保留）。…
- L230（印刷區）：…¦ HV4SITL16 ¦ 120.0 ¦ 5/270°／無 ¦ 330.14 ¦ 1260.4→12…
- L238（印刷區）：…速、場址上限 8.0 m/s，W ≥ 6.16 m/s 即 NO_GO。(d) HV4SITL16 與 HV4SITL15（未列）為同計畫重飛（完工時間差 0.025 s），模擬…
- L345（附錄）：…¦ HV4SITL16 未設風：`sitl_cmdlines.txt` 無 `SIM_WIND`；巡航…
- L381（附錄）：…3. 補 HV4SITL16 的報表時**一定要設 `HV_TRIAL=HV4SITL16`**，否則 `s…
- L383（附錄）：…字（本 label 不能改圖）**：`fig3_results.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 …

### `+0.02 s`（K6 四捨五入假影）：全檔 1 處，其中印刷區（第 1、2 節）0 處
- L312（附錄）：…¦ 「降天花板的代價只有 +0.02 s」 ¦ 「+0.0279 s」或「1260.4261 → 1260.4540 s…

### `4000 m`（決定 3）：全檔 1 處，其中印刷區（第 1、2 節）0 處
- L320（附錄）：…¦ 「Wang 等人以轉彎半徑 4000 m…」 ¦ 不寫該數字 ¦ 未經原文核對（決定 3） ¦…

### `0.008 m/s`（K7 舊值）：全檔 0 處，其中印刷區（第 1、2 節）0 處

### `68/68`（舊參數數）：全檔 1 處，其中印刷區（第 1、2 節）0 處
- L301（附錄）：…¦ 「參數讀回 68/68」 ¦ 「102/102（34 項 × 3 機）」 ¦ 舊數字只比了 17 個參…

### `120.10`（舊最高高度）：全檔 2 處，其中印刷區（第 1、2 節）0 處
- L17（第 0 節排版指示（不印））：… HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更正（見附錄 D） ¦…
- L383（附錄）：…lts.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 m），本版已自排版移除，若要恢復三聯圖須以 HV4…

### `0/310`（舊違規分母）：全檔 2 處，其中印刷區（第 1、2 節）0 處
- L17（第 0 節排版指示（不印））：…ord**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更正（見附錄 D） …
- L383（附錄）：…3_results.png` 內嵌 HV4SITL16 之舊值（101.0 m、0/310、120.10／120.12／120.16 m），本版已自排版移除，若要恢復三聯…

### `未設風`（C2：只能出現在描述歷史對照／BEAMIDP1 之處）：全檔 4 處，其中印刷區（第 1、2 節）2 處
- L238（印刷區）：… 與 HV4SITL15（未列）為同計畫重飛（完工時間差 0.025 s），模擬未設風，與 119.5 m 各趟差兩個變因。(e) 規劃間距為 2 s 網格上界，較實…
- L258（印刷區）：…三機完成起降、最小間距 180.77 m、高於 120 m 之時間 0 s；模擬未設風，其完工時間不引用）。…
- L345（附錄）：…¦ HV4SITL16 未設風：`sitl_cmdlines.txt` 無 `SIM_WIND`；巡航段東西向…
- L374（附錄）：…出與實飛最高相對高度（HV4SITL15／16 為 120.0 m 天花板且模擬未設風之歷史對照；單變因對照 HV4W55R 見表3），(b) 規劃最小水平間距（2 …

