# AASRC 2026 加長型摘要 — 給 Cowork 的排版素材包（HARVEST，中文稿）

> 本檔是**可直接排版的完整內容**。請 Cowork 依第 0 節的指示，在官方格式檔上排出 ≤4 頁的 Word 稿。
> 內文的每個數字都已從原始飛行紀錄或程式碼重算並經獨立審查（出處見附錄 B）。**請勿改動數字，也不要把措辭「潤飾」得更強**（見附錄 A 禁用措辭）。

同資料夾檔案：

| 檔案 | 用途 |
|---|---|
| `AASRC2026-投稿格式_加長型摘要_.doc` | 官方格式檔（從這份開始編輯） |
| `fig1_architecture.png`（.svg） | 圖1 系統架構，300 dpi，寬 17.0 cm |
| `fig2_flowchart.png`（.svg） | 圖2 規劃演算法流程，300 dpi，寬 17.0 cm |
| `fig3_results.png`（.svg） | 圖3 SITL 飛行結果，300 dpi，寬 17.0 cm |
| `figures_src/` | 產圖腳本與圖3 數據出處（`fig3_data.json`），不放進 Word |

---

## 0. 排版指示（給 Cowork）

### 0.1 格式規定（官方格式檔原文整理）

- Word 編輯，**單行間距**；內文 **11 點**；中文一律**標楷體**，英文與數字 **Times New Roman**。
- **A4**，上下左右各留 **2 cm**（版心寬 17 cm）；**頁碼於頁尾置中**。
- 加長型摘要**全文不超過 4 頁**，請直接以官方格式檔編輯。
- 中文或英文擇一。本稿用**中文**，所以**刪除格式檔後半的整段英文範本**（從 “English Title” 到英文 REFERENCES 結束）。
- 中文題目 14 點粗體置中；作者、服務單位 12 點置中；「摘要」「誌謝」「參考文獻」12 點粗體。
- **題目／作者姓名、服務單位／摘要之間各空一行。**
- 數學式用 **Word 內建方程式**製作，式號 (1)…(5) 靠右。做法：插入 → 方程式，貼上 2.3 節提供的線性格式後按 Enter 轉成專業格式；式號用定位點，或用無框線的三欄表格。
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
| 第 3 頁 | 2.3 核心公式 (1)–(5)＋表1；三、SITL 驗證：3.1 設置、3.2 結果＋表2 |
| 第 4 頁 | 圖3；3.3 起降走廊判準對照＋表3；四、結論與限制；誌謝；參考文獻 |

實際斷頁以內容自然流動為準，不要硬塞分頁符號。

### 0.4 超過 4 頁時的刪減順序（依序做到剛好 4 頁為止）

> ⚠ 實測：本稿原樣以 Word 套官方格式檔排版、**未取消文件格線**時約 **6 頁**。取消格線（0.2 節）後的頁數尚未量測，預期仍可能超過 4 頁，請先取消格線，再依下列順序刪減。

1. 2.1、2.2 節文字各只保留第一句（「系統架構如圖1。」「規劃流程如圖2。」後接圖）。
2. 表1 改成內文一句話：「Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m，可用航程 133–145／79–86 km，兩型共用 HFOV 60° 相機。」
3. 3.3 節刪文字，只留表3 與表註。

**不可刪**：摘要、表2、圖3、第四節的「限制」、所有表註（表註是 caveat，刪了就是誤導）。

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

本文針對法規高度上限下之異質垂直起降（VTOL）無人機隊測繪任務，提出 HARVEST 規劃流程。在 120 m 作業高度下，小型複合翼 VTOL 之轉彎半徑（120–200 m）與條帶間距（約 97 m）同量級，且每架次皆含起降與場坪排隊之固定成本。HARVEST 結合含風時間成本、依轉彎半徑與可用能量之決策級機數配額、Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離產生任務，並匯出至 ArduPilot QuadPlane。以 1 架 Pioneer 與 2 架 HERO 覆蓋 1.248 km² 作業區進行 SITL 閉迴路飛行，三機皆完成起降，航跡回算覆蓋率 100%（上界），最高高度貼齊 120 m 上限（最大超出 0.16 m），航程較規劃多 2.3%，最小水平間距 101.0 m（門檻 100 m），飛控參數讀回 102/102 相符。另一受控對照顯示，走廊豁免若以場坪座標而非場坪身分判定，最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。

**關鍵詞：** 覆蓋路徑規劃、異質無人機隊、垂直起降固定翼、續航約束任務配置、四維隔離

---

## 2. 正文

### 一、前言

小型複合翼垂直起降（VTOL）無人機兼具定點起降與長航程巡航，適合區域測繪；異質機隊可依任務分派構型，但規劃須同時處理轉彎半徑、續航、起降場坪與飛航高度。依民用航空法第99條之14，遙控無人機未經核准時，飛航高度不得逾距地面或水面四百呎[1]，本文取 120 m 為上限；此時相機刈幅約 138.6 m、條帶間距約 97.0 m，與小型 VTOL 之規劃轉彎半徑（120–200 m）同量級。

覆蓋路徑規劃以胞分解與來回掃描為基礎[2]，DARP 將區域分給外部給定數量之機器人[3]，Wang 等人[4]則以轉彎半徑 4000 m 之同質固定翼機隊驗證協同覆蓋規劃。含風時間成本[5]、曲率受限路徑[6]、迴轉與跳帶排序[7,8]、機數閉式界[9]、逐機不同轉彎半徑[10]、異質機隊多區覆蓋[11]、逐機不同起降點[12]，以及 QuadPlane 之能量感知覆蓋規劃[13]均已有文獻。本文不主張單一元件為新，而是提出 HARVEST（Heterogeneous Allocation and Routing for VTOL Endurance-constrained Survey Tasks），將其整合為可輸出至 ArduPilot 飛控[14]之流程。貢獻為：(i) 以各構型含風有效轉彎半徑計算作業時間，並以決策級機數配額取代僅作為搜尋種子之機數界；(ii) 將 VTOL 起降能量、轉換高度與法規高度帶納入規劃；(iii) 以場坪身分為鍵之起降走廊互斥，搭配具時間不確定度之四維隔離。本文並以 SITL 閉迴路飛行與讀回式檢核驗證其可執行性。

### 二、研究方法

#### 2.1 系統架構

系統架構如圖1。輸入與介面層由 AeroPlan 地面站或情境腳本提供作業區、機隊、空域上限、風與場坪；HARVEST 規劃層產生每機四維航點，並進行去衝突與整體驗證；ArduPilot 執行與驗證層匯出 QuadPlane 任務與參數並上傳 SITL，逐項讀回參數、比對任務項數，不符即停飛，飛行後以航跡回算覆蓋率與間距。本文之驗證實驗均經情境腳本路徑執行。

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

取 $h$ = 120 m、$\alpha_H$ = 60°、$o_s$ = 0.30，得 $w$ = 138.6 m、$s$ = 97.0 m。時間成本採含風地速[5]與有效半徑 $R_{eff}$ 之 Dubins 路徑[6]：

$$V_g(\theta)=\sqrt{V_a^2-W_\perp^2}+W_\parallel,\qquad t=\sum_{line}\frac{L}{V_g(\theta)}+\sum_{arc}\sum_{j=1}^{n}\frac{R_{eff}\,\delta}{V_g(\theta_j)} \tag{3}$$

Word 線性格式：
```
V_g(θ)=√(V_a^2−W_⊥^2)+W_∥,  t=∑_line L/V_g(θ)+∑_arc ∑_(j=1)^n (R_eff δ)/V_g(θ_j)
```

$W_\parallel$、$W_\perp$ 為風速之航跡切向與法向分量，$\delta$ 為弧段步長（≤5°）。當 $s<2R_{eff}$ 時，跳帶數依 span coefficient $k=\lceil 2R_{eff}/s\rceil$ 決定[7,8]（公式值：Pioneer 5、HERO 3）。各胞機數以決策級配額決定：

$$\lambda_{floor}=\left\lceil\frac{T_{single}}{\kappa_{load}\,T_{use}-t_{in}-t_{RTB}}\right\rceil,\qquad T_{use}=\frac{3600\,E_{use}}{P_c\,k_{unc}} \tag{4}$$

Word 線性格式：
```
λ_floor=⌈T_single/(κ_load T_use−t_in−t_RTB)⌉,  T_use=3600 E_use/(P_c k_unc)
```

$T_{single}$ 為單機飛完該胞（含全部迴轉）之時間，$t_{in}$、$t_{RTB}$ 為進場與返場時間，$\kappa_{load}$ = 0.85、$k_{unc}$ = 1.2075，$E_{use}$ 已扣除 30% 保留與起降能量。不同於 Gong 等人[9]作為搜尋種子之 $K_{lower}=L_{in}/L_{max}$，本式直接作為配置輸入；因拆分後胞內迴轉次數減少，$\lambda_{floor}$ 並非機數下界。四維隔離與起降走廊互斥定義為：

$$\begin{aligned}&\mathrm{conflict}(a,b)\iff d_h<d_{thr}\ \wedge\ d_v<\Delta h_{sep}\ \wedge\ |t_a-t_b|<\Delta t+\sigma_t(t_a)+\sigma_t(t_b),\quad \sigma_t(t)=\min\{k_t\max(0,t-t_{sync}),\ \sigma_{\max}\}\\&\mathrm{site}(r_a)=\mathrm{site}(r_b)\ \wedge\ u_a\neq u_b\ \Rightarrow\ I(r_a)\cap I(r_b)=\varnothing\end{aligned} \tag{5}$$

Word 線性格式（兩行，可用方程式陣列，或分兩個方程式共用式號）：
```
d_h<d_thr∧d_v<Δh_sep∧|t_a−t_b|<Δt+σ_t (t_a)+σ_t (t_b),  σ_t (t)=min(k_t max(0,t−t_sync),σ_max)
site(r_a)=site(r_b)∧u_a≠u_b⇒I(r_a)∩I(r_b)=∅
```

$d_{thr}$ 預設 100 m（兩機同向作業時為 60 m）、$\Delta h_{sep}$ = 50 m、$\Delta t$ = 2 s、$k_t$ = 0.05、$\sigma_{\max}$ = 45 s；$I(r)$ 為場坪預約時窗（起飛 60 s；降落為進場前 30 s 至落地後 90 s）。

**表1 兩型 VTOL 平台規劃參數**（表標題置於表上方置中）

| 參數 | Pioneer（A 構型） | HERO（B 構型） |
|---|---|---|
| 巡航空速 | 21.5 m/s | 19.0 m/s |
| 轉彎半徑（理論／規劃，最大坡度） | 130／200 m（20°） | 75／120 m（25°） |
| 續航／可用航程（含 30% 保留） | 176 min／133–145 km | 136 min／79–86 km |
| 最低固定翼作業高度／空域上限 | 100 m／120 m AGL | 100 m／120 m AGL |
| 相機（兩型共用） | HFOV 60°、側疊 30% ⇒ 刈幅 138.6 m、間距 97.0 m | 同左 |

表註：規格為公開測試值與規劃值；電池能量、相機焦距（16 mm）與像素間距（3.9 μm）為假設值。

### 三、SITL 驗證與成果

#### 3.1 實驗設置

作業區為 1560 m × 800 m 平地矩形（1.248 km²），由系統自動切為 3 個子區；機隊為 1 架 Pioneer（A1）與 2 架 HERO（B2、B3），起飛坪相距 10 m，共用降落場坪 H；空域上限 120 m AGL 不申請豁免，最低固定翼作業高度 100 m，目標 GSD 0.05 m/px，規劃假設 5 m/s 西風。飛行採 ArduPlane 4.8.0-dev QuadPlane SITL（內建機體模型、5 倍速），依規劃起飛時刻逐機放行。覆蓋率以航跡加刈幅緩衝回算，間距以 2 s 網格內插航跡計算，兩者均為上界。

#### 3.2 規劃與 SITL 飛行對照

結果如表2 與圖3。三機完成起降，高度貼齊 120 m 上限，航程較規劃多 2.2%–2.5%。最小水平間距 101.0 m 發生於 B2–B3（垂直差 29.0 m），雖無違規，但距門檻僅 1.0 m；同一計畫重飛為 104.2 m。參數讀回 102 筆中，含 3 筆韌體相容之刻意覆寫（Q_RTL_MODE）與 2 筆整數截斷（AIRSPEED_MAX）。

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

> 【插入圖3：`fig3_results.png`，寬 17.0 cm，置中】
> 圖說：見附錄 C

#### 3.3 起降走廊判準對照

於 1 架 Pioneer 與 3 架 HERO、起飛坪相距 10 m 之情境，只切換走廊豁免判準（表3）。以座標判定時，相距 10 m 之不同場坪被視為同坪而豁免四維檢查，最小間距降至 5.0 m／19.9 m（B3–B4 起降段），規劃驗證卻仍通過、覆蓋率仍為 100%；改以場坪身分判定則無違規。此單一受控情境顯示，場坪語意與飛行安全直接相關，且覆蓋率僅為必要而非充分之驗收條件。

**表3 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**

| 豁免判準 | 最小水平／垂直間距 | 違規樣本 | 規劃驗證 | 覆蓋率 |
|---|---|---|---|---|
| 場坪座標（≤50 m 視為同坪） | 5.0／19.9 m | 5／378 | 通過（衝突 0） | 100% |
| 場坪身分（site） | 52.7／112.6 m | 0／346 | 通過（衝突 0） | 100% |

表註：回收站自動指派關閉；間距為 2 s 網格內插上界；單一情境、單次執行。

### 四、結論與限制

HARVEST 將含風時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane；SITL 閉迴路飛行中，三機完成起降、高度貼齊上限、參數讀回全數相符，走廊判準對照則顯示覆蓋率不足以作為唯一驗收條件。

本研究之限制：(1) 僅以 SITL 內建機體模型驗證，且模擬未設風；(2) 覆蓋率與間距為航跡回算上界，主結果間距餘裕僅 1.0 m，且同型 HERO 之間無垂直隔離通道；(3) 驗證情境每胞條帶數少於 3，跳帶排序未被觸發；(4) 以最長單機航程衡量時，HARVEST 在封閉基準之 10 組情境中均較團隊既有之 DCCPP 實作長 0.9%–22.9%，本文不主張航程效率；(5) 各條件僅 1–2 趟、單一作業區形狀。後續將設定模擬風場重飛，並進行硬體上模擬（Simulation on Hardware）與轉彎半徑消融實驗。

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

---

## 附錄 A　禁用措辭（Cowork 潤稿時請勿改成左欄）

| 禁用 | 應寫成 | 理由 |
|---|---|---|
| 首次、首創、全新演算法 | 「本文不主張單一元件為新」「整合」 | 各元件均有先行文獻 [5]–[13] |
| λ_floor 是「下界」「最少機數」「保證不少派」 | 「決策級機數配額」 | 拆分後迴轉減少，可能大於真正最小值；程式的 t_in、t_RTB 取最小值 |
| 「我們提出 ⌈2R/s⌉／跳帶係數」 | 「依 span coefficient [7,8]」 | Huang 2022 命名，Bochtis 2008 已有 2R/w 門檻 |
| 「航程較短」「較快」「效率優於基準」 | 不寫；限制 (4) 已誠實交代 | 封閉基準 10 組中最長單機航程均較長 |
| 「實飛」「實機驗證」 | 「SITL 飛行」「SITL 閉迴路驗證」 | 機體物理為 SITL 內建模型 |
| 「規劃與實飛時間只差數秒」 | 必須附「SITL 未設風」表註 | 東西向誤差會互相抵銷，時間吻合不代表排程正確 |
| 「參數讀回 68/68」 | 「102/102（34 項 × 3 機）」 | 舊數字只比了 17 個參數 |
| 「任務逐項讀回相同」 | 「任務項數讀回一致」 | 程式只比項數 |
| 「民航局規定 120 m」 | 「法規四百呎，本文取 120 m」 | 條文為四百呎（121.92 m） |
| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表3 即反例 |
| 「證明走廊互斥與四維隔離皆與安全相關」 | 「單一受控情境顯示場坪判準影響間距」 | 對照只切換走廊豁免判準，σ_t 未做消融 |
| 「以 skip-5／skip-3 飛行」 | 「公式值 k = 5／3」 | 本次情境每胞條帶數 <3，未觸發 |

## 附錄 B　關鍵數字出處（供核對，不放進 Word）

路徑相對 repo 根目錄 `aeroplan-studio/`。

| 數字 | 出處 |
|---|---|
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

參考文獻 DOI（Word 稿不必印，供核對用）：
[2] 10.1023/A:1008958800904　[3] 10.1007/s10846-016-0461-x　[4] 10.1016/j.dt.2025.01.008　[5] 10.3390/s18072132　[6] 10.2307/2372560　[7] 10.1016/j.biosystemseng.2008.06.008　[8] 10.3390/aerospace9090521　[9] 10.3390/math14030416　[10] 10.1007/s10846-016-0458-5　[11] 10.3390/drones7110664　[12] 10.3390/drones9090658　[13] 10.3390/drones9040287

## 附錄 C　圖說定稿

圖說置於圖下緣置中，11 點，各一行：

- 圖1 HARVEST 系統架構與本文 SITL 驗證路徑（粗箭頭）
- 圖2 HARVEST 規劃演算法流程（灰底點劃框為不可行或未覆蓋出口，虛線為再分配回圈）
- 圖3 SITL 飛行結果：(a) 規劃與飛行航跡，(b) 高度剖面，(c) 同時在空之機間間距

## 附錄 D　投稿前待辦（給使用者，不是給 Cowork）

1. 填寫作者、服務單位、國科會計畫編號；確認大會版面（單欄、版心 17 cm）與截稿日。
2. **證據出處不乾淨**：HV4SITL16 與 PAD8 兩臂都是在「HEAD 5e0295f + 未提交修改」下飛的。HV4SITL16 未納入 git；PAD8_* 被 `.gitignore:109` 排除，而且兩臂的建置身分只能靠 `docs/hv_dccpp/VENDOR.md` 的敘述證明。依專案紀律，這類數字不應進正式論文 ⇒ 建議在乾淨 commit 上重飛這三趟並納管，數字若變動再更新表2、表3、圖3。
3. 補 HV4SITL16 的報表時**一定要設 `HV_TRIAL=HV4SITL16`**，否則 `scripts/hv_sitl_report.py` 會覆寫未納管的 HV4SITL15 產物。
4. 最小間距只比門檻多 1.0 m，而且是 HERO–HERO 同高度帶。審稿人可能會追問，建議準備設風重飛與加密遙測（≥2 Hz）的補充數據。
