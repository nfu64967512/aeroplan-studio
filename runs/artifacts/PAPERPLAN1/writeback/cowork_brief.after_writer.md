# AASRC 2026 加長型摘要 — 給 Cowork 的排版素材包（HARVEST，中文稿）

**v2026-09-21，對應 commit 3442fcb（證據 95a337b、腳本 ff38a77）**

> 本檔是**可直接排版的完整內容**。請 Cowork 依第 0 節的指示，在官方格式檔上排出 ≤4 頁的 Word 稿。
> 內文的每個數字都已從原始飛行紀錄或程式碼重算並經獨立審查（出處見附錄 B；本版依指導老師審閱意見 C1–C7 改寫，飛行數字一律以 `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json` 為準）。**請勿改動數字，也不要把措辭「潤飾」得更強**（見附錄 A 禁用措辭）。

同資料夾檔案：

| 檔案 | 用途 |
|---|---|
| `AASRC2026-投稿格式_加長型摘要_.doc` | 官方格式檔（從這份開始編輯） |
| `fig1_architecture.png`（.svg） | 圖1 系統架構，300 dpi，寬 17.0 cm |
| `fig2_flowchart.png`（.svg） | 圖2 規劃演算法流程，300 dpi，寬 17.0 cm |
| `fig5_compliance.png`（.svg） | **圖3** 高度合規與實飛間距，300 dpi，寬 17.0 cm（檔名為 fig5，於正文為圖3） |
| `fig4_wind.png`（.svg） | **備用圖4** 風場對照，300 dpi，寬 17.0 cm；**本版未排入**，Word 實測有餘時才接在圖3 之後（見 0.4） |
| `fig3_results.png`（.svg） | ⚠ **本版不放進 Word**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更正（見附錄 D） |
| `figures_src/` | 產圖腳本與數據出處（`fig3_data.json`、`fig4_data.json`、`fig5_data.json`），不放進 Word |

---

## 0. 排版指示（給 Cowork）

### 0.1 格式規定（官方格式檔原文整理）

- Word 編輯，**單行間距**；內文 **11 點**；中文一律**標楷體**，英文與數字 **Times New Roman**。
- **A4**，上下左右各留 **2 cm**（版心寬 17 cm）；**頁碼於頁尾置中**。
- 加長型摘要**全文不超過 4 頁**，請直接以官方格式檔編輯。
- 中文或英文擇一。本稿用**中文**，所以**刪除格式檔後半的整段英文範本**（從 “English Title” 到英文 REFERENCES 結束）。
- 中文題目 14 點粗體置中；作者、服務單位 12 點置中；「摘要」「誌謝」「參考文獻」12 點粗體。
- **題目／作者姓名、服務單位／摘要之間各空一行。**
- 數學式用 **Word 內建方程式**製作，式號 (1)…(6) 靠右。做法：插入 → 方程式，貼上 2.3 節提供的線性格式後按 Enter 轉成專業格式；式號用定位點，或用無框線的三欄表格。
- **表標題置於表上方置中**（「表1 …」）；**圖標題置於圖下緣置中**（「圖1 …」）。
- 頁首沿用格式檔原樣：「2026中華民國航太學會學術研討會／虎尾，中華民國115年11月14日／2026 AASRC Conference／Huwei, November 14, 2026／論文編號：YY-XX」。論文編號由大會給號，先保留 YY-XX。

### 0.2 本稿的補充排版約定

- 章標題（一、二、三、四）：12 點粗體標楷體，靠左。小節標題（2.1 等）：11 點粗體。
- 參考文獻：11 點；英文用 Times New Roman，中文法規條目用標楷體；以 [1]…[n] 編號，使用凸排。
- **圖一律以 17.0 cm 原寬插入，不要縮放**：圖內最小字高在 17 cm 時剛好 8 點。版面不夠時請依 0.4 的刪減順序處理，不要縮圖。
- 圖設為「與文字排列」（inline），置中。
- **所有段落（含表格、圖說、參考文獻）取消「文件格線貼齊」**：段落 → 縮排與行距 → 取消勾選「文件格線被設定時，貼齊格線」。格式檔設有每行 18 點的文件格線，不取消的話 11 點字每行被撐成 18 點，一頁只剩約 40 行；格式檔自己的說明段落也是取消格線的設定。行距仍維持「單行間距」。
- 表格：11 點、單行間距、儲存格上下邊距 0；框線用三線表（上、表頭下、下），不加直線。

### 0.3 版面配置（目標 4 頁）

| 頁 | 內容 |
|---|---|
| 第 1 頁（封面頁） | 頁首、題目、作者、服務單位、**摘要**、關鍵詞、一、前言 |
| 第 2 頁 | 二、研究方法：2.1 系統架構＋圖1；2.2 規劃流程＋圖2 |
| 第 3 頁 | 2.3 核心公式 (1)–(6)＋表1；三、SITL 驗證：3.1 設置、3.2 主結果＋表2 |
| 第 4 頁 | 3.3 天花板與風場對照＋表3＋圖3；3.4 起降走廊判準對照＋表4；3.5 跳帶排序觸發情境；四、結論與限制；誌謝；參考文獻 |

實際斷頁以內容自然流動為準，不要硬塞分頁符號。

### 0.4 超過 4 頁時的刪減順序（依序做到剛好 4 頁為止）

> ⚠ 實測：v2026-09-14 稿原樣以 Word 套官方格式檔排版、**未取消文件格線**時約 **6 頁**；取消格線（0.2 節）後的頁數尚未量測。本版（v2026-09-21）為回應審閱意見 C1–C7 新增 3.3、3.5 節與表3，以字數估計取消格線後約 5.9 頁（同法估 v2026-09-14 稿約 4.2 頁，該稿依其刪減順序做完約 3.9 頁；估計法見 `runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md`；此估計法以舊稿實測校準約偏低 15%）。請先取消格線，再依下列順序刪減，直到 4 頁為止。

1. 2.1、2.2 節文字各只保留第一句（「系統架構如圖1。」「規劃流程如圖2。」後接圖）。
2. 表1 改成內文一句話：「Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m（規劃裕度可吸收之風速 W* = 5.1675／5.0333 m/s），可用航程 133–145／79–86 km（估計值），兩型共用 HFOV 60° 相機。」表1 之表註保留，接在該句之後。
3. 3.4 節刪文字，只留表4 與表註。
4. 3.5 節只留「另設觸發情境…均為規劃層模型值。」一句。
5. 表3 刪 HV4SITL16 與「W ≥ 6.16 m/s」兩列（表註 (c)(d) 保留）。

⚠ 1–5 項全做後估計仍約 5.4 頁。**以下 6–8 項會刪掉實質內容，須由使用者決定，Cowork 不可自行執行**（全做後約 4.9 頁）：

6. 圖1 改用同資料夾之 `fig1_architecture_compact.png`（同內容，高 5.3 cm；原圖 6.8 cm）。
7. 刪圖2 與其圖說，並把 2.2 節首句「規劃流程如圖2。」改為「規劃流程如下。」（2.2 節其餘文字保留）。
8. 3.4 節與表4 改為一句：「另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定則為 52.7 m 且無違規，而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管）。」

**不可刪**：摘要、表2、表3、圖3、第四節的「限制」、所有表註（表註是 caveat，刪了就是誤導）。備用圖4（`fig4_wind.png`）與附錄 E 之備用 3.6 節（表5）只有在 Word 實測仍不足 4 頁時才排入。

---

## 1. 封面資訊

### 題目（14 點粗體置中）

**主推：** HARVEST：法規高度限制下異質垂直起降無人機隊之續航約束測繪規劃與SITL驗證

備選一：法規高度限制下異質垂直起降無人機隊之續航約束測繪規劃
備選二：異質垂直起降無人機隊測繪任務之續航約束配置、起降走廊互斥與SITL閉迴路驗證

### 作者與服務單位（12 點置中，**待使用者填寫**）

作者一ᵃ、作者二ᵃ、作者三ᵇ
ᵃ服務單位（中文）
ᵇ服務單位（中文）

### 摘要（「摘要」12 點粗體置中；內文 11 點左右對齊）

本文針對法規高度上限下之異質垂直起降（VTOL）無人機隊測繪任務，提出 HARVEST 規劃流程。在 120 m 作業上限下，小型複合翼 VTOL 之轉彎半徑（120–200 m）與條帶間距（約 96.6 m）同量級，且每架次皆含起降與場坪排隊之固定成本。HARVEST 結合考慮風場之飛行時間成本、依轉彎半徑與可用能量之決策級機數配額、Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離產生任務，並匯出至 ArduPilot QuadPlane。以 1 架 Pioneer 與 2 架 HERO 覆蓋 1.2434 km² 作業區，於 5 m/s 模擬風場中進行 SITL 閉迴路飛行，三機皆完成起降，航跡回算覆蓋率 99.97%（上界），全程最高高度 119.68 m AGL，低於 120 m 上限（規劃天花板 119.5 m），航程較規劃多 3.7%–4.7%，最小水平間距 100.43 m（門檻 100 m），飛控參數讀回與韌體正規化後之期望值 102/102（34 項 × 3 機）相符。單變因對照顯示，規劃天花板若取 120.0 m，實飛累計 354.93 s 高於 120 m。另一受控對照顯示，走廊豁免若以場坪座標而非場坪身分判定，最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。

**關鍵詞：** 覆蓋路徑規劃、異質無人機隊、垂直起降固定翼、續航約束任務配置、四維隔離

---

## 2. 正文

### 一、前言

小型複合翼垂直起降（VTOL）無人機兼具定點起降與長航程巡航，適合區域測繪；異質機隊可依任務分派構型，但規劃須同時處理轉彎半徑、續航、起降場坪與飛航高度。依民用航空法第99條之14，遙控無人機未經核准時，飛航高度不得逾距地面或水面四百呎[1]，本文取 120 m 為作業上限、119.5 m 為規劃天花板；於後者，相機刈幅約 138.0 m、條帶間距約 96.6 m，與小型 VTOL 之規劃轉彎半徑（120–200 m）同量級。

覆蓋路徑規劃以胞分解與來回掃描為基礎[2]，DARP 將區域分給外部給定數量之機器人[3]，Wang 等人[4]則以同質固定翼機隊驗證複雜區域之協同覆蓋規劃。考慮風場之飛行時間成本[5]、曲率受限路徑[6]、迴轉與跳帶排序[7,8]、機數閉式界[9]、逐機不同轉彎半徑[10]、異質機隊多區覆蓋[11]、逐機不同起降點[12]，以及 QuadPlane 之能量感知覆蓋規劃[13]均已有文獻。本文不主張單一元件為新，而是提出 HARVEST（Heterogeneous Allocation and Routing for VTOL Endurance-constrained Survey Tasks），將其整合為可輸出至 ArduPilot 飛控[14]之流程。貢獻為：(i) 以各構型之風修正有效轉彎半徑[15]與考慮風場之地速計算作業時間，並以決策級機數配額取代僅作為搜尋種子之機數界；(ii) 將 VTOL 起降能量、轉換高度與法規高度帶納入規劃；(iii) 以場坪身分為鍵之起降走廊互斥，搭配具時間不確定度之四維隔離。本文並以 SITL 閉迴路飛行與讀回式檢核驗證其可執行性。

### 二、研究方法

#### 2.1 系統架構

系統架構如圖1。輸入與介面層由 AeroPlan 地面站或情境腳本提供作業區、機隊、空域上限、風與場坪；HARVEST 規劃層產生每機四維航點，並進行去衝突與整體驗證；ArduPilot 執行與驗證層匯出 QuadPlane 任務與參數並上傳 SITL，逐項讀回參數與模擬風設定、比對任務項數，不符即停飛，飛行後以航跡回算覆蓋率與間距。本文之驗證實驗均經情境腳本路徑執行。

> 【插入圖1：`fig1_architecture.png`，寬 17.0 cm，置中】
> 圖說（圖下緣置中）：見附錄 C

#### 2.2 規劃流程

規劃流程如圖2。通過前置檢查後，將作業區分解為凸胞並鋪設條帶（兩構型高度差不足時改以時間分離），依決策級配額配置構型與架數，以 Beam-IDP 排序條帶並檢查返場能量；最後組裝起降走廊與三維剖面，偵測並修復四維與走廊衝突後輸出任務檔。

> 【插入圖2：`fig2_flowchart.png`，寬 17.0 cm，置中】
> 圖說：見附錄 C

#### 2.3 核心公式

規劃目標為最小化機隊完工時間（makespan），各項依序為起飛、進場、作業、轉移、返場與降落時間：

$$\min C_{\max}=\max_i C_i,\qquad C_i=t_i^{TO}+T_i^{in}+\textstyle\sum T_i^{oper}+\sum T_i^{trans}+T_i^{RTB}+T_i^{LDG} \tag{1}$$

Word 線性格式：
```
min C_max=max_i C_i,  C_i=t_i^TO+T_i^in+∑T_i^oper+∑T_i^trans+T_i^RTB+T_i^LDG
```

條帶幾何由作業高度 $h$ 與相機水平視角 $\alpha_H$ 決定，作業高度受空域上限 $h_{ceil}$ 與最低固定翼高度 $h_{\min fw}$ 夾限：

$$w=2h\tan\frac{\alpha_H}{2},\qquad s=w\,(1-o_s),\qquad h_{op}=\max\big(h_{\min fw},\ \min(h_{tgt},h_{\max},h_{ceil})\big) \tag{2}$$

Word 線性格式：
```
w=2h tan(α_H/2),  s=w(1−o_s),  h_op=max(h_(min fw),min(h_tgt,h_max,h_ceil))
```

取 $h$ = 119.5 m（規劃天花板）、$\alpha_H$ = 60°、$o_s$ = 0.30，得 $w$ = 138.0 m、$s$ = 96.6 m。時間成本以考慮風場之地速[5]與風修正有效轉彎半徑 $R_{eff}$ 之 Dubins 路徑[6]計算，直線段與圓弧段以同一地速式逐段積分：

$$V_g(\theta)=\sqrt{V_a^2-W_\perp^2}+W_\parallel,\qquad t=\sum_{line}\frac{L}{V_g(\theta)}+\sum_{arc}\sum_{j=1}^{n}\frac{R_{eff}\,\delta}{V_g(\theta_j)} \tag{3}$$

Word 線性格式：
```
V_g(θ)=√(V_a^2−W_⊥^2)+W_∥,  t=∑_line L/V_g(θ)+∑_arc ∑_(j=1)^n (R_eff δ)/V_g(θ_j)
```

$W_\parallel$、$W_\perp$ 為風速沿航跡之切向與法向分量（完整風三角），$\delta$ 為弧段積分步長（≤5°）。有效轉彎半徑取定常風中轉彎地面軌跡（trochoid，次擺線[5,15]）之順風相位曲率半徑，並以規劃半徑為下限：

$$R_{eff,c}(W)=\max\Big\{R_{plan,c},\ R_{theo,c}\big(1+\tfrac{W}{V_{a,c}}\big)^2\Big\},\qquad R_{theo,c}=\frac{V_{a,c}^2}{g\tan\varphi_{max,c}},\qquad W^*_c=V_{a,c}\Big(\sqrt{\tfrac{R_{plan,c}}{R_{theo,c}}}-1\Big) \tag{4}$$

Word 線性格式：
```
R_(eff,c)(W)=max{R_(plan,c), R_(theo,c)(1+W/V_(a,c))^2},  R_(theo,c)=V_(a,c)^2/(g tan φ_(max,c)),  W_c^*=V_(a,c)(√(R_(plan,c)/R_(theo,c))−1)
```

此為既有結果[15]，非本文之貢獻；本文僅取此最壞相位曲率半徑作為地面座標之單一 Dubins 半徑，使條帶排序與成本表全程留在地面座標。$W$ 為作業高度帶內之最大層風速，$W^*$ 為規劃半徑之內建裕度可吸收之風速。當 $s<2R_{eff}$ 時，跳帶數依 span coefficient $k=\lceil 2R_{eff}/s\rceil$ 決定[7,8]（本文情境 W = 5 m/s 低於 $W^*$，$R_{eff}=R_{plan}$，k 為 Pioneer 5、HERO 3）。各胞機數以決策級配額決定：

$$\lambda_{floor}=\left\lceil\frac{T_{single}}{\kappa_{load}\,T_{use}-t_{in}-t_{RTB}}\right\rceil,\qquad T_{use}=\frac{3600\,E_{use}}{P_c\,k_{unc}} \tag{5}$$

Word 線性格式：
```
λ_floor=⌈T_single/(κ_load T_use−t_in−t_RTB)⌉,  T_use=3600 E_use/(P_c k_unc)
```

$T_{single}$ 為單機飛完該胞（含全部迴轉）之時間，$t_{in}$、$t_{RTB}$ 為進場與返場時間，$\kappa_{load}$ = 0.85、$k_{unc}$ = 1.2075，$E_{use}$ 已扣除 30% 保留與起降能量。不同於 Gong 等人[9]作為搜尋種子之 $K_{lower}=L_{in}/L_{max}$，本式直接作為配置輸入；因拆分後胞內迴轉次數減少，$\lambda_{floor}$ 並非機數下界。四維隔離與起降走廊互斥定義為：

$$\begin{aligned}&\mathrm{conflict}(a,b)\iff d_h<d_{thr}\ \wedge\ d_v<\Delta h_{sep}\ \wedge\ |t_a-t_b|<\Delta t+\sigma_t(t_a)+\sigma_t(t_b),\quad \sigma_t(t)=\min\{k_t\max(0,t-t_{sync}),\ \sigma_{\max}\}\\&\mathrm{site}(r_a)=\mathrm{site}(r_b)\ \wedge\ u_a\neq u_b\ \Rightarrow\ I(r_a)\cap I(r_b)=\varnothing\end{aligned} \tag{6}$$

Word 線性格式（兩行，可用方程式陣列，或分兩個方程式共用式號）：
```
d_h<d_thr∧d_v<Δh_sep∧|t_a−t_b|<Δt+σ_t (t_a)+σ_t (t_b),  σ_t (t)=min(k_t max(0,t−t_sync),σ_max)
site(r_a)=site(r_b)∧u_a≠u_b⇒I(r_a)∩I(r_b)=∅
```

$d_{thr}$ 預設 100 m（兩機同向作業時為 60 m）、$\Delta h_{sep}$ = 50 m、$\Delta t$ = 2 s、$k_t$ = 0.05、$\sigma_{\max}$ = 45 s；$I(r)$ 為場坪預約時窗（起飛 60 s；降落為進場前 30 s 至落地後 90 s）。

**表1 兩型 VTOL 平台規劃參數**（表標題置於表上方置中）

| 參數 | Pioneer（A 構型） | HERO（B 構型） |
|---|---|---|
| 巡航空速 $V_a$ | 21.5 m/s | 19.0 m/s |
| 轉彎半徑 $R_{theo}$／$R_{plan}$（最大傾角 $\varphi_{max}$） | 130／200 m（20°） | 75／120 m（25°） |
| 規劃裕度可吸收之風速 $W^*$（式 (4)） | 5.1675 m/s | 5.0333 m/s |
| 續航／可用航程（含 30% 保留） | 176 min／133–145 km | 136 min／79–86 km |
| 最低固定翼作業高度／空域上限 | 100 m／120 m AGL | 100 m／120 m AGL |
| 相機（兩型共用） | HFOV 60°、側疊 30% ⇒ 於 119.5 m 刈幅 137.99 m、間距 96.59 m | 同左 |

表註：規格為公開測試值與規劃值；電池能量、相機焦距（16 mm）與像素間距（3.9 μm）為假設值，續航與可用航程為估計值。$R_{theo}$ 為廠商值（幾何自洽值 129.46／78.92 m）；以表列值計，HERO 於 W = 5 m/s 之順風相位需傾角 26.07°，略大於 25°。

### 三、SITL 驗證與成果

#### 3.1 實驗設置

作業區為標稱 1560 m × 800 m 之平地矩形（WGS84 橢球面積 1.2434 km²），由系統自動切為 3 個子區；機隊為 1 架 Pioneer（A1）與 2 架 HERO（B2、B3），起飛坪相距 10 m，共用降落場坪 H；空域上限取法規四百呎（121.92 m）以下之 120 m AGL，不申請豁免，規劃天花板設為 119.5 m 以保留匯出離散化與高度追蹤之裕度（3.3 節）；最低固定翼作業高度 100 m，目標 GSD 0.05 m/px。規劃風與模擬風設為獨立變因（表3），主結果兩者均為 5 m/s、270°；模擬風於解鎖前讀回確認（三機 15 項），紊流為 0、不隨高度變化。飛行採 ArduPlane 4.8.0-dev QuadPlane SITL（內建機體模型、5 倍速），依規劃起飛時刻逐機放行。覆蓋率以航跡加刈幅緩衝回算，為上界；間距以飛控原生 5.1–6.4 Hz 紀錄於 WGS84 橢球座標求連續時間最小值；完工時間以最後一架下穿 2 m 判定。

#### 3.2 規劃與 SITL 飛行對照

結果如表2（架次 HV4W55）。三機完成起降，全程最高 119.675 m AGL、高於 120 m 之時間 0 s；最小水平間距 100.43 m（B2–B3）僅餘 0.43 m（見限制 (2)）。參數每機比對 34 項：任務產生器匯出 32 項，另 2 項（`ARMING_REQUIRE`、`SCHED_LOOP_RATE`）為 SITL 平台設定而非韌體需求；「相符」指與韌體正規化後期望值之差 ≤ max(0.01, 0.1%)，正規化含 `Q_RTL_MODE` 刻意覆寫（3 機，韌體不接受其與 `RTL_AUTOLAND` 同時啟用）與 `AIRSPEED_MAX` 整數化（2 機）；對原始規劃值（96 項）則到 float32 精度為 91／96、逐位元為 85／96。

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

**表4 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**

| 豁免判準 | 最小水平／垂直間距 | 違規樣本 | 規劃驗證 | 覆蓋率 |
|---|---|---|---|---|
| 場坪座標（≤50 m 視為同坪） | 5.0／19.9 m | 5／378 | 通過（衝突 0） | 100% |
| 場坪身分（site） | 52.7／112.6 m | 0／346 | 通過（衝突 0） | 100% |

表註：回收站自動指派關閉；間距為 2 s 網格內插上界（既有報表程式之量尺，非 3.1 節之方法）；單一情境、單次執行；兩臂原始資料（`PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW`）未納入版本控制，第三方目前無從核對。

#### 3.5 跳帶排序之觸發情境

表2 情境之各胞條帶數（Pioneer 1、HERO 2）未超過 k，排序無作用（限制 (3)）。另設觸發情境（作業區改為三塊標稱 1600 × 1200 m，其餘同表2；各胞 n = 6–13 > k），以規劃器自身之考慮風場 Dubins 胞內成本評分：全機隊迴轉時間由順序掃描之 1696.88 s 降為 1115.73 s（−34.25%），距 Held–Karp 最佳解仍高 5.24%，均為規劃層模型值。交錯航線之可執行性另經 SITL 飛行確認（BEAMIDP1：三機完成起降、最小間距 180.77 m、高於 120 m 之時間 0 s；模擬未設風，其完工時間不引用）。

### 四、結論與限制

HARVEST 將考慮風場之飛行時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane。5 m/s 模擬風場之 SITL 閉迴路飛行中三機完成起降、最高高度低於 120 m、參數讀回 102／102 相符；單變因對照顯示規劃天花板降 0.5 m 即可消除 354.93 s 之超高飛行，代價 +0.0279 s；納入風場換得的是預測準確度而非較短之完工時間；走廊判準對照則顯示覆蓋率不足以作為唯一驗收條件。

本研究之限制：(1) 僅以 SITL 內建機體模型驗證；風場為單層、定常、無紊流；W ≥ 6.16 m/s 即 NO_GO，風場驗證上限為 5 m/s（情境預設值，非場址實測）。(2) 覆蓋率為上界；間距餘裕僅 0.426 m（八趟最緊 0.291 m），且最近接時刻垂直差 28.4 m 低於 50 m 門檻，水平間距為唯一防線，同型 HERO 間無垂直隔離通道；規劃間距為上界，不蘊含實飛 ≥ 100 m。(3) 跳帶排序之觸發條件為單胞條帶數 n > k = ⌈2R_eff/s⌉，而非「條帶數少於 3」；表2 情境 n = 1／2 未超過 k = 5／3，四種條帶序之迴轉成本逐項相同，此缺口源自切區結果；3.5 節之效果為模型值，順序掃描對照組無法實飛。(4) 最長單機航程於封閉基準 10 組情境均較團隊既有之 DCCPP 實作長 0.91%–22.87%，本文不主張航程效率；另 2 組 HARVEST 自判不可行而回報未覆蓋（覆蓋率 0.528、0.617），恰為基準航線超出配置器能量預算 12.5% 與 14.4% 者，其成因（切區後才計算每胞機數而不再細切）列為後續工作。(5) W = 5 m/s 低於 W* = 5.1675／5.0333 m/s，R_eff 退化為 R_plan（匯出之迴轉半徑逐趟為 200／120 m），其風修正僅在 W* 至 6.15 m/s 生效，本文未驗證。(6) 條帶方位不隨風向調整；270° 與 180° 架次為各自重解之不同計畫。(7) 圓弧匯出離散化缺陷（3.3 節）以降低天花板迴避而未修正：匯出偏差 +0.014 至 +0.170 m、飛控追蹤另 +0.027 至 +0.148 m，此裕度不可外推（最壞餘裕 0.303 m）。(8) 各條件僅 1 趟、單一作業區，SITL 為決定性模擬。後續將進行硬體上模擬、陣風與分層風對照及轉彎半徑消融實驗。

### 誌謝

本研究由國家科學及技術委員會計畫（計畫編號：NSTC ○○○-○○○○-E-○○○-○○○）補助，特此致謝。
（**待使用者填寫**；若無計畫補助，整節刪除。）

### 參考文獻

[1] 民用航空法，第九章之二「遙控無人機」，第99條之14，總統華總一義字第11200054081號令修正公布，2023年6月28日。
[2] Choset, H., "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition", Autonomous Robots, Vol. 9, No. 3, pp 247-253, December 2000.
[3] Kapoutsis, A. C., Chatzichristofis, S. A., and Kosmatopoulos, E. B., "DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning", Journal of Intelligent & Robotic Systems, Vol. 86, No. 3-4, pp 663-680, June 2017.
[4] Wang, M., Zhang, D., Li, C., and Zhang, Z., "Multiple Fixed-Wing UAVs Collaborative Coverage 3D Path Planning Method for Complex Areas", Defence Technology, Vol. 47, pp 197-215, May 2025.
[5] Coombes, M., Fletcher, T., Chen, W.-H., and Liu, C., "Optimal Polygon Decomposition for UAV Survey Coverage Path Planning in Wind", Sensors, Vol. 18, No. 7, Article 2132, July 2018.
[6] Dubins, L. E., "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents", American Journal of Mathematics, Vol. 79, No. 3, pp 497-516, July 1957.
[7] Bochtis, D. D., and Vougioukas, S. G., "Minimising the Non-Working Distance Travelled by Machines Operating in a Headland Field Pattern", Biosystems Engineering, Vol. 101, No. 1, pp 1-12, September 2008.
[8] Huang, J., Fu, W., Luo, S., Wang, C., Zhang, B., and Bai, Y., "A Practical Interlacing-Based Coverage Path Planning Method for Fixed-Wing UAV Photogrammetry in Convex Polygon Regions", Aerospace, Vol. 9, No. 9, Article 521, September 2022.
[9] Gong, H., Fu, Z., Xu, K., Sun, W., Xu, W., and Du, M., "Bi-Level Optimization for Multi-UAV Collaborative Coverage Path Planning in Irregular Areas", Mathematics, Vol. 14, No. 3, Article 416, 2026.
[10] Sundar, K., and Rathinam, S., "Algorithms for Heterogeneous, Multiple Depot, Multiple Unmanned Vehicle Path Planning Problems", Journal of Intelligent & Robotic Systems, Vol. 88, No. 2-4, pp 513-526, 2017.
[11] Xiao, P., Li, N., Xie, F., Ni, H., Zhang, M., and Wang, B., "Clustering-Based Multi-Region Coverage-Path Planning of Heterogeneous UAVs", Drones, Vol. 7, No. 11, Article 664, November 2023.
[12] Yakunin, K., Kuchin, Y., Muhamedijeva, E., Symagulov, A., and Mukhamediev, R. I., "An Algorithm for Planning Coverage of an Area with Obstacles with a Heterogeneous Group of Drones Using a Genetic Algorithm and Parameterized Polygon Decomposition", Drones, Vol. 9, No. 9, Article 658, September 2025.
[13] Mathur, A., and Atkins, E., "Multi-Mode Flight Simulation and Energy-Aware Coverage Path Planning for a Lift+Cruise QuadPlane", Drones, Vol. 9, No. 4, Article 287, April 2025.
[14] ArduPilot Dev Team, "QuadPlane Overview", ArduPilot Plane Documentation, https://ardupilot.org/plane/docs/quadplane-overview.html, accessed September 14, 2026.
[15] Techy, L., and Woolsey, C. A., "Minimum-Time Path Planning for Unmanned Aerial Vehicles in Steady Uniform Winds", Journal of Guidance, Control, and Dynamics, Vol. 32, No. 6, pp 1736-1746, 2009.

---

## 附錄 A　禁用措辭（Cowork 潤稿時請勿改成左欄）

| 禁用 | 應寫成 | 理由 |
|---|---|---|
| 首次、首創、全新演算法 | 「本文不主張單一元件為新」「整合」 | 各元件均有先行文獻 [5]–[13]、[15] |
| λ_floor 是「下界」「最少機數」「保證不少派」 | 「決策級機數配額」 | 拆分後迴轉減少，可能大於真正最小值；程式的 t_in、t_RTB 取最小值 |
| 「我們提出 ⌈2R/s⌉／跳帶係數」 | 「依 span coefficient [7,8]」 | Huang 2022 命名，Bochtis 2008 已有 2R/w 門檻 |
| 「航程較短」「較快」「效率優於基準」 | 不寫；限制 (4) 已誠實交代 | 封閉基準 10 組中最長單機航程均較長 |
| 「實飛」「實機驗證」 | 「SITL 飛行」「SITL 閉迴路驗證」 | 機體物理為 SITL 內建模型 |
| 「規劃與實飛時間只差數秒」／「含風規劃較快」 | 「含風規劃之預測誤差較小（+1.70% 對 +2.43%），但完工時間反而長 47.13 s，其中 44.0 s 是刻意延後放行」 | 含風規劃買到的是預測準確度；百分比對 t0 判準敏感，須連同表2 表註引用 |
| 「參數讀回 68/68」 | 「102/102（34 項 × 3 機）」 | 舊數字只比了 17 個參數 |
| 「參數讀回 102／102 相符」（不給定義）／「原始值嚴格比對 91／96」 | 「102／102 與韌體正規化後期望值相符；到 float32 精度之原始值比對為 91／96，逐位元為 85／96」 | 判準 max(0.01, 0.1%)；6 筆為 float32 往返表示誤差 |
| 「任務逐項讀回相同」 | 「任務項數讀回一致」 | 程式只比項數 |
| 「民航局規定 120 m」 | 「法規四百呎，本文取 120 m」 | 條文為四百呎（121.92 m） |
| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表4 即反例 |
| 「證明走廊互斥與四維隔離皆與安全相關」 | 「單一受控情境顯示場坪判準影響間距」 | 對照只切換走廊豁免判準，σ_t 未做消融 |
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

## 附錄 B　關鍵數字出處（供核對，不放進 Word）

路徑相對 repo 根目錄 `aeroplan-studio/`。

| 數字 | 出處 |
|---|---|
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

參考文獻 DOI（Word 稿不必印，供核對用）：
[2] 10.1023/A:1008958800904　[3] 10.1007/s10846-016-0461-x　[4] 10.1016/j.dt.2025.01.008　[5] 10.3390/s18072132　[6] 10.2307/2372560　[7] 10.1016/j.biosystemseng.2008.06.008　[8] 10.3390/aerospace9090521　[9] 10.3390/math14030416　[10] 10.1007/s10846-016-0458-5　[11] 10.3390/drones7110664　[12] 10.3390/drones9090658　[13] 10.3390/drones9040287　[15] 10.2514/1.44580（未複核）

## 附錄 C　圖說定稿

圖說置於圖下緣置中，11 點：

- 圖1 HARVEST 系統架構與本文 SITL 驗證路徑（粗箭頭）
- 圖2 HARVEST 規劃演算法流程（灰底點劃框為不可行或未覆蓋出口，虛線為再分配回圈）
- 圖3（檔案 `fig5_compliance.png`）高度合規與實飛間距：(a) 各趟匯出與實飛最高相對高度（HV4SITL15／16 為 120.0 m 天花板且模擬未設風之歷史對照；單變因對照 HV4W55R 見表3），(b) 規劃最小水平間距（2 s 網格上界）與實飛值（WGS84 橢球連續時間最小值；× 為既有平面近似值）
- 備用圖4（檔案 `fig4_wind.png`，本版未排入）風場對照：(a) 巡航段逐航向之（地速 − 空速）中位數，東−西差約為 2W；(b) 規劃風／模擬風組合下之機隊完工時間

## 附錄 D　投稿前待辦（給使用者，不是給 Cowork）

1. 填寫作者、服務單位、國科會計畫編號；確認大會版面（單欄、版心 17 cm）與截稿日。
2. **證據出處（v2026-09-21 更新）**：八趟飛行證據（HV4SITL15／16、HV4W00／05／55／55R／55S、BEAMIDP1）已於 95a337b 納管，重算程式與文字包於 3442fcb 納管。仍未乾淨者：(a) 各趟 `.BIN` 未納管，故「.BIN → `out/*.json`」一段只有持有 log 者能重跑；(b) 表4 兩臂 `PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW` 仍被 `.gitignore:109` 排除（表4 已加註第三方無從核對）；(c) 附錄 E 表5 航程列之 `project_rows_5e0295f5_dirty.csv` 未納管、`bench.py` 為已修改狀態；(d) **本資料夾（含本檔與 fig4／fig5）尚未納管**，檔頭版本戳所指 commit 不含本檔；納管後請更新版本戳。
3. 補 HV4SITL16 的報表時**一定要設 `HV_TRIAL=HV4SITL16`**，否則 `scripts/hv_sitl_report.py` 會覆寫未納管的 HV4SITL15 產物。
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
