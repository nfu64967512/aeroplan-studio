# HV-DCCPP 統一規格書（Heterogeneous-VTOL Dynamic Complete Coverage Path Planning, v1.0）

構型 A = Pioneer PNP（5 kg LiDAR）、構型 B = HERO PNP（0.6 kg 測繪相機），皆為 lift-and-cruise 4+1 複合翼 VTOL。基礎：Wang et al. 2025, *Defence Technology* 47:197–215（下稱「論文」）。平台數值一律取自 `platform_params.md` 表 1（不得更改），任務輸入為表 2。本規格書為三份設計草案（幾何／任務／實作）與評審意見之合併裁決版，為後續 Python 參考實作（`hv_dccpp/`，見 `module_contract.md`）的唯一權威來源。

---

## 0 裁決紀錄（Decision log）

所有草案間的分歧與評審指出的錯誤，在此一次裁決；後文不再重複討論。

| # | 議題 | 裁決 |
|---|---|---|
| D1 | 風中轉彎半徑 | $R_{eff}=\max\{R_{plan},R_{theo}(1+W/V_a)^2\}$（擺線最大曲率半徑）；Dubins **直接在地面座標**以 $R_{eff}$ 生成。否決「$R_{plan}(1+W/V_a)^2$」（過度保守）與「空速座標系生成再轉換」（端點固定於地面時為移動目標問題，不可靜態轉換）。 |
| D2 | 坡度受限包絡 | 前向遞迴用**下降角**、後向遞迴用**爬升角**、取 max；簡報 2.1(2) 的兩趟命名對調，以本規格為準。加起／終點釘住、雙向上界 $u$、不可行偵測。 |
| D3 | 條帶寬度 | 以條帶走廊（橫向 $\pm w/2$）內最高地形之最小 AGL $h_{\min,l}$ 決定；寬度迭代採「超集帶單調收縮」（任一次迭代終止皆保守）；最後一條溢出以整體置中平移處理（不改變間距）。 |
| D4 | 轉移狀態配對 | 四個端點狀態明確定義（§8.2）：$S^{LR}_{l}=(p^R_l,\theta_{LR})\to S^{R}_{l'}=(p^R_{l'},\theta_{RL})$，$S^{RL}_{l}=(p^L_l,\theta_{RL})\to S^{L}_{l'}=(p^L_{l'},\theta_{LR})$。 |
| D5 | 環型（LSL）轉場長度 | $s<2R$ 時為 $3\pi R+(2R-s)$（$s=0$：$11.42R$）；Ω/RLR 為 $R(\pi+4\arccos\frac{s+2R}{4R})$，U 型（$ks\ge2R$）為 $\pi R+(ks-2R)$。三式皆為單元測試值，規劃時一律由 6-word Dubins 求解器自動得出。 |
| D6 | 能量模型 | 單一 Wh 模型（§5）：$P_c^{ref}=E_{nom}/T_{ref}$、$(m/m_{ref})^{1.5}$、$k_{unc}=k_{bm}k_{as}=1.15\times1.05$、保留 $r E_{nom}$ 只扣一次、逐航點備降可達；`usable_distance_km` 為獨立上限。否決「分率模型 + 保留雙重扣除」與「加扣 10% $E_{nom}$」。 |
| D7 | 氣象門檻 | 放飛：$W_{go}=\min(8.0,0.45V_a)$（A 8.0、B 8.0）、陣風 12 m/s、能見度 ≥3 km、降雨 ≤0.5 mm/h、溫度 −10…40 °C；中止：平均風 >10 m/s 或陣風 >14 m/s 或能見度 <1.5 km 或降雨 >2 mm/h。Beaufort 5（8.0–10.7 m/s）為能力宣稱，不作放行門檻。否決「$W\le0.5(V_{tr}-V_s)=1$ m/s」。 |
| D8 | airspeed_sensor_validated=false | 轉換空速指令 $V_{tr}+2.0$ m/s、轉換完成高度指令取區間上緣、$k_{as}=1.05$、站點陣風限 8 m/s（驗證後 10）、側風限 6/5 m/s（驗證後 8/6）。 |
| D9 | HERO min_mission_airspeed 15.0–16.0 | 規劃用 16.0；15.0 僅作遙測絕對下限。失速裕度檢查 $V_{a,\min}\ge1.2V_s\sqrt{m/m_{ref}}$。 |
| D10 | max_airspeed_mps | 不用於規劃，僅作 $V_a^*$ 上限檢查；HERO 為 `None`（待實機確認）時跳過檢查並標示 provisional。否決 1.25/1.4 倍 fallback。 |
| D11 | c2_link_range_km、terrain_clearance_m「待填入」 | 必填任務輸入，缺則 NO-GO；不給靜默預設。建議 $h_{clr}\ge3\sigma_{DEM}+30$ m，DEM 誤差已知時強制。 |
| D12 | Pioneer 120 m AGL 下限 vs 120 m AGL 法規上限 | 高度帶政策（§4.6）：每胞高度帶寬 $\Delta h_{band}\ge20$ m 才可指派該構型；`waiver_max_agl_m` 豁免旗標可提高上限；無豁免且平地 → A 對該胞不可行（warning），改由 B。否決「僅允許 120 AGL 可飛」。 |
| D13 | obstacle_horizontal_buffer_m | 只用於起降點檢查；航路 keep-out 水平緩衝另設 $b_h$（預設 50 m，含追蹤誤差）。 |
| D14 | 弧段時間 | 一律沿弧以 $V_g(\theta)$ 數值積分（Δθ=5°）；$L/V_a$ 非保守（Jensen：圓周平均 $1/V_g\ge1/V_a$，W=5 m/s 時 A 低估 4.3%）。快速代理用 $L/V_a\cdot(1+(W/V_a)^2)$。 |
| D15 | 方向評分坡度懲罰 | 有限懲罰（§7.3）取代 ∞；所有方向皆超坡時仍可規劃並標記，交由 §11.5 處置。 |
| D16 | $\lambda$ 正規化 | 每構型獨立 Eq.31，$\lambda\in[0,\min(n_U^c,n_q^c)]$ 加能量下限；$\lambda=0$ 之胞由拍賣（一機多胞）承接；胞數 > 機數為常態。 |
| D17 | 混編胞 | 同一胞不允許 A、B 同時鋪條帶；`any` 寬胞以殘帶分割成 A 子胞＋B 子胞；`both` 生成兩套獨立條帶集。 |
| D18 | 分層與微分層 | A/B 分層 $\Delta h_{sep}=50$ m；否決同胞奇偶條帶 ±10 m 微分層。同胞多機以 IDP 剪枝（相鄰條帶時間重疊）＋側向間距 $s\ge d_{sep,lat}=60$ m ＋ 4D 檢查。 |
| D19 | 降落轉換起點 | 在降落點**下風側**：$P_{tc}=T-d_{tr}\hat t(\theta_{app})$，$\theta_{app}=\beta$（逆風進場）。 |
| D20 | 懸停／轉換參數 | $k_h=4.0$、$P_{tr}=P_h+P_c$、$t_{tr}^{out}=20$ s、$t_{tr}^{in}=25$ s、$v_{z,up}=2.0$、$v_{z,dn}=1.0$ m/s（最後 10 m 0.5）、$d_{tr}$ 由公式 $V_{tr,cmd}^2/(2a_{tr})+V_{tr,cmd}t_{hold}$（未驗證 468/384、驗證 384/308 m；`transition_ground_run_m` 可覆寫），皆 provisional。 |
| D21 | 當架次 AUW → 爬升差異 | $\tan\gamma_c(m)=\tan\gamma_c^{plan}\cdot\mathrm{clip}(1-k_\gamma(m/m_{ref}-1),\,0.6,\,1)$，$k_\gamma=2.0$；下降角不變；失速裕度隨 $\sqrt{m/m_{ref}}$。 |
| D22 | 時間窗 | 三種：區域時間窗（配置硬限制 H3）、空域全域時間窗（所有起降須在窗內）、禁區時間窗（重疊任務窗即視為有效）。 |
| D23 | 氣象中止 | 獨立事件 WEATHER_ABORT（§12.9），含降落排序（能量餘裕優先）與上風側等待。 |
| D24 | DEM/DSM | `is_dsm=True` 時不再燒入 tree/building 類障礙（避免雙重緩衝），tower/powerline 仍燒入；keep-out 對所有障礙建立。 |
| D25 | 重疊率預設 | 任務輸入；預設 LiDAR 側向 0.25，相機側向 0.30、前向 0.70。 |
| D26 | 失鏈 | 每機策略 `lost_link_policy`，$t_{ll}=60$ s；地面以預載動作推算 4D 航跡並讓其他機避讓（§12.4）。 |
| D27 | Dubins 離散 | 外切多邊形角步 $\Delta\psi=2\arccos\big(R/(R+e_{\max})\big)$（$e_{\max}=3$ m），上限 30°。 |
| D28 | IDP | k-nearest $K=6$ ＋ beam $B=3$ ＋ 去重；階段成本 = makespan 下界（含 $\hat t_{RTB}$）＋ $\epsilon$ 總和 ＋ $\mu$ 論文相對項（$\tau_0=60$ s 防除零）。複雜度以 $K^\lambda$ 陳述，不再宣稱「beam 後 $O(\beta\lambda)$」。 |
| D29 | transition_complete_agl 區間取用 | 指令值：驗證後取下緣、未驗證取上緣；爬升距離依指令值；能量一律用上緣；走廊淨空一律用下緣。 |
| D30 | 障礙繞行 | 主：圓角偏移多邊形（$\rho=\max(b_h,R_{eff})$）切線圖 Dijkstra；備：可視圖 + 頂點緩衝 $R_{eff}(\sec(\Delta/2)-1)$。 |
| D31 | DP 簡化只外擴 | $P=\mathrm{simplify}(Z\oplus\mathrm{disc}(\delta_{edge}),\varepsilon)$，$\varepsilon=0.1\min_c s_c<\delta_{edge}$ ⇒ $Z\subseteq P$（DP 誤差 ≤ ε 之保證）。 |
| D32 | 論文退化 | `legacy_mode=True` 一鍵設定全部退化條件（§15）。 |
| D33 | IDP 分支結構（審查 v1.1） | 由論文的鎖步（lockstep，每階段每機各取一條）改為**事件驅動**：每階段只由 $T_i$ 最小的機選下一條（K 候選 × 4 種轉場、beam $B$），階段數 $=n$，複雜度 $O(nBK)$；各機條帶數可不等。`legacy_mode` 保留鎖步。 |
| D34 | 同向轉場（racetrack） | 轉移矩陣擴為 4 型（§8.2）：反向 $S^{LR}_l\to S^R_{l'}$、$S^{RL}_l\to S^L_{l'}$ 與同向 $S^{LR}_l\to S^L_{l'}$、$S^{RL}_l\to S^R_{l'}$；論文 Eq.(25) 放寬為「作業方向 = 轉移終端狀態方向」，Eq.(26) 保留。單向可行（順風超 $V_g^{cap}$）的胞因此可覆蓋。 |
| D35 | CLIMB／DESCENT 段 | $P_{tc}$ 後至首次達 $z_g^{\max}+h_{\min fw}$ 為 CLIMB 段、對稱之 DESCENT 段：$z_{lo}$ 不含 $h_{\min fw}$（表 1 之 $H_{tc}<h_{\min fw}$ 使其必要），只含 $h_{clr}$ 與障礙；$h_{\min fw}$ 只對 ENTRY/OPER/TRANS/RTB 生效。 |
| D36 | OPER 段下界含 $h_{\min,l}$ | `sample_bounds` 對 OPER 樣本 $z_{lo}\leftarrow\max(z_{lo},z_g^{\max}{}_{w/2}+h_{\min,l})$，使 $z^*$ 本身保證條帶寬度，平滑投影不可能破壞（§11.4）。 |
| D37 | 轉換段幾何 | 轉換段（TRANSITION_OUT/IN）為**旋翼輔助**飛行（$P_{tr}=P_h+P_c$），$H_{ts}\leftrightarrow H_{tc}$ 的高度變化由旋翼承擔，不受固定翼 3°/4° 限制但受 $\gamma_{tr,\max}$（provisional 8°）；固定翼坡度限制自 $P_{tc}$ 起／至 $P_{tc}$ 止。另加 FW_ACCEL／FW_DECEL 直線段（$V_{tr,cmd}\leftrightarrow V_{a,c}$）。`transition_profile="fw_slope"` 時改為延長 $d_{tr}$ 至 $(H_{tc,hi}-H_{ts})/\tan\gamma$。 |
| D38 | 迴轉帶幾何 | 端線方向外延 $d_{turn}=\max\{R_{eff},\sqrt{4R_{eff}^2-(R_{eff}+s/2)^2}+R_{eff}\}+e_{ext}$（Ω 超出量），側向外延 $R_{eff}-s/2$（僅 $s<2R$ 之首末條帶）；用於 §7.3 禁區檢查、§7.6 $s^{turn}$、H2 鏈路、§6.2 裁切內縮、§10.2/§13 剪枝。 |
| D39 | 基準電池 | $P_c^{ref}=f_{dep}E_{ref}/T_{ref}$，$E_{ref}$=`reference_battery_wh`（執行 176/136 min 基準測試之電池，provisional；缺省 = 當架次標稱並警告）；保留 $rE_{nom}$ 仍用當架次標稱。 |
| D40 | 落點政策 | RTB 一律先試 $H$（或指定回收點）；只有 $H$ 不可行才依序試備降點並標 `planned_landing_at_alternate`；$C_i$ 以 $H$ 計；多架次要求第一架次落於 $H$。 |
| D41 | 禁區緩衝 | $P=P\setminus\bigcup(\text{NoFly}\oplus b_{trk})$；胞外擴 $\delta_{edge}$ 後再 $\cap$ 該集合（禁區邊界不外擴）；作業路徑（$\oplus b_{trk}$，含 $e_{ext}$）逐條檢查禁區。 |
| D42 | 高度基準 | 所有內部函式一律 ENU $z$；`GeoFrame.enu_z_to_msl / msl_to_enu_z` 負責轉換；MSL 輸入（風層、空域、障礙頂、禁區樓層）於解析時轉為 ENU $z$。 |
| D43 | 感測器兩層 | 硬體 `SensorHardware`（焦距、像元、FOV、點率、測距、快門上限）與任務需求 `PayloadRequirement`（GSD／密度／重疊率／快門率／目標高）分離；`sensor_model(hw, req)`；區域需求覆寫硬體預設。預設硬體值為示例（`is_default_example`）並警告。 |

---

## 1 問題定義與假設

### 1.1 問題敘述

給定：
- 機隊 $\mathcal U=\mathcal U_A\cup\mathcal U_B$，每架 $U_i$ 有構型 $c(i)\in\{A,B\}$、當架次 AUW $m_i$、可用電量 $E^{avail}_i$（Wh）、感測器規格。
- 任務區域 $\mathcal Z=\{Z_k\}$：簡單多邊形（可凹、可含洞），附感測需求 $\sigma_k\in\{\text{lidar},\text{rgb},\text{both},\text{any}\}$、優先度 $val_k$、時間窗 $[t^{open}_k,t^{close}_k]$、酬載需求（GSD／點密度／重疊率）。
- 環境：DEM/DSM、障礙物（點／線／面＋高度）、高度分層風場、C2 鏈路覆蓋、空域（高度上限、禁區、時間窗）、氣象門檻與觀測。
- 起飛點 $H$、備降點集合 $\mathcal A$（`alternate_landing_sites_required=true` → $|\mathcal A|\ge1$）。

求：每架機一條完整架次航跡 $\Gamma_i$ = 起飛走廊 → 進入路徑 → (作業路徑, 轉移路徑)* → 返航路徑 → 降落走廊，三維、時間參數化、附能量與模式旗標，使

$$\min\ C_{\max}=\max_i C_i,\qquad C_i=t^{TO}_i+T^{in}_i+\sum T^{oper}_i+\sum T^{trans}_i+T^{RTB}_i+T^{LDG}_i$$

（min-max 落地時刻，含返航；次要目標 $\sum_i C_i$ 作 tie-break），受限於：
- (C1) 覆蓋：每區依 $\sigma_k$ 被對應構型足跡完全覆蓋，每條作業路徑恰被一機掃過一次（論文 Eq.19–21）。
- (C2) 運動學：地面曲率半徑 $\ge R_{eff}$；地面坡度 $\in[-\tan\gamma^g_d,\tan\gamma^g_c]$；空速 $\in[V_{a,\min},V_{a,c}]$。
- (C3) 高度：$z\in[z_{lo},z_{hi}]$（地形＋淨空、障礙頂＋垂直緩衝、最低固定翼高度、感測器上限、空域上限）。
- (C4) 能量：至承諾點 $P_{tc}$ 為止任一航點 $E^{rem}\ge rE_{nom}+\min_a e^{alt}$；$E(P_{tc})\ge rE_{nom}+e_{MA}$（missed approach）；落地餘量 $\ge rE_{nom}$。
- (C5) 鏈路／半徑：所有航點在 C2 覆蓋內且距基地 $\le\rho^{sym}_c$（下緣）。
- (C6) 空域：禁區、時間窗。
- (C7) 去衝突：高度分層或 4D 分離；起降走廊互斥。
- (C8) 氣象：放飛門檻與任務中止條件。

### 1.2 假設（承襲論文並修正）

| # | 論文假設 | 本規格 |
|---|---|---|
| A1 | 同構固定翼 | 兩構型；參數相同且關閉新限制時退化為論文（§15） |
| A2 | 起始於區外、定速 | 起始狀態 = 轉換完成點（或爬升盤旋出口）；空速固定 $V_{a,c}$，作業段可降至 $V_{a,\min}$；地速由風三角決定 |
| A3 | FOV 前視梯形固定 | LiDAR 扇形／相機矩形足跡，寬度以條帶保守最小 AGL 計算；梯形保留為 `fov_model="trapezoid"` |
| A4 | 升降時地速不變 | 升降時空速不變（$\dot z=V_a\sin\gamma$ 與風無關）；能量另計爬升功率 |
| A5 | 一機一區 | 一機可依序覆蓋多個凸胞；IDP 在團隊條帶集合上執行 |
| A6 | 無風、無能量、無障礙 | 分層風場（規劃週期內準靜態）；能量硬限制含 RTB；點／線／面 keep-out |
| A7 | — | 懸停僅用於垂直起降、轉換、緊急；**任務中所有轉彎皆為固定翼 Dubins（$R_{eff}\ge R_{plan}$）**，不存在懸停定點轉向 |
| A8 | — | 感測器僅在作業路徑上開機；進入／轉移／返航段不計覆蓋 |
| A9 | — | 幾何在本地 ENU（原點 = 主起飛點）計算；ENU 之 $z$ 已含地球曲率，論文 Eq.35 不需另作補償 |

### 1.3 兩層偵查概念與本案關鍵尺度

| | 第一層（A, Pioneer） | 第二層（B, HERO） |
|---|---|---|
| 責任區 | 遠距（$\rho^{sym}$ 66 km）、大面積、寬條帶、LiDAR 需求、長時間佇留 | 近距（39 km）、小面積／狹窄／凹角胞、RGB/GSD 需求、目標再確認（cue-and-confirm） |
| 優勢來源 | 可用時間 $\approx0.53\times176$ min、足跡 210 m @150 m AGL | 轉彎半徑 120 m → 窄胞轉場浪費少 |

分工由 §9 的構型適配度與硬限制自動導出，非固定。**關鍵尺度**：論文 $R=4000$ m、$w=2200$ m（$R\gg w$ 不成立但 $R\ll$ 條帶長）；本案 $R\in[120,200]$ m 與條帶間距 $s\in[100,160]$ m 同量級，$s<2R$ 為常態，故「燈泡（Ω）轉場 vs skip-$k$ 交錯順序」成為主要效率來源，是論文未遇到的情境。

---

## 2 符號表

| 符號 | 意義 | 單位 |
|---|---|---|
| $c\in\{A,B\}$ | 構型 | — |
| $V_{a,c},V_{a,\min,c},V_{tr,c},V_{s,c}$ | 巡航／任務下限／轉換／失速空速（21.5/18.0/16.0/14.0；19.0/16.0/14.0/12.0） | m/s |
| $V_a^*$ | 航段實際空速（§4.4） | m/s |
| $\theta$ | 航跡方位（ENU，自北順時針）；$\hat t=(\sin\theta,\cos\theta)$，$\hat n=(\cos\theta,-\sin\theta)$ | rad |
| $\beta,W,\vec w$ | 風來向、風速、風向量 $\vec w=-W(\sin\beta,\cos\beta)$ | rad, m/s |
| $W_\parallel,W_\perp$ | $\vec w\cdot\hat t$、$\vec w\cdot\hat n$ | m/s |
| $V_g(\theta)$ | 地速 | m/s |
| $R_{plan},R_{theo},R_{eff}$ | 規劃／理論／風修正有效轉彎半徑（200/130；120/75） | m |
| $\phi_{\max}$ | 最大傾角（20°/25°） | deg |
| $\gamma_c,\gamma_d$ | 規劃爬升／下降空氣坡度（3°/4°） | deg |
| $\gamma^g_c(\theta),\gamma^g_d(\theta)$ | 對應地面坡度 | deg |
| $m_i,m_{ref,c}$ | 當架次 AUW、基準 AUW（24.6/6.65） | kg |
| $H_{ts},H_{tc}$ | 轉換起始／完成 AGL（60；80–100 ／ 50；70–90） | m |
| $h_{\min fw,c}$ | 最低固定翼作業 AGL（120/100） | m |
| $h_{clr}$ | 地形淨空（任務必填） | m |
| $h_{ceil}$ | 空域 AGL 上限（含豁免） | m |
| $h_{op},h_{\min,l}$ | 目標作業 AGL；條帶 $l$ 的保守最小 AGL | m |
| $w_c(h),s_c(h)$ | 足跡寬（swath）、條帶間距 | m |
| $o_{side},o_{fwd}$ | 側向／前向重疊率 | — |
| $e_{ext}$ | 條帶端點外延 | m |
| $\delta_{edge},b_{trk},b_h,b_v$ | 邊界外擴、航跡半寬、keep-out 水平／垂直緩衝 | m |
| $Z_k,Q_{k,q}$ | 區域、凸胞 | — |
| $\alpha_q$ | 胞掃描方位 | rad |
| $L_{q,l},p^L,p^R$ | 作業路徑、左右端點 | — |
| $S=(x,y,\theta,R)$ | Dubins 狀態；$D(S_m\to S_n)$ Dubins 曲線 | — |
| $\vec t^{oper},\vec t^{in},\vec t^{trans}$ | 二元作業／進入／轉移時間（論文 Eq.10–12 之時間版） | s |
| $\mathbf T_{i},\mathbf E_i$ | 時間矩陣（論文 $D_{i,k}$）、決策矩陣 | s, {−1,0,1} |
| $E_{nom},E^{avail},E^{use},E^{rem}$ | 標稱／可用／可規劃／剩餘能量 | Wh |
| $P_c,P_h,P_{tr},P_\uparrow,P_\downarrow$ | 巡航／懸停／轉換／爬升／下降功率 | W |
| $r$ | reserve_fraction = 0.30 | — |
| $k_{unc}=k_{bm}k_{as}$ | 未驗證保守係數 | — |
| $e^{alt}(x,a)$ | 自狀態 $x$ 至備降點 $a$ 之能量 | Wh |
| $\sigma_k$ | 感測需求 | — |
| $S^c_q,J^c_q,\lambda^c_q$ | 適配度、配置指標、指派架數 | — |
| $C_i,C_{\max}$ | 完工時刻、makespan | s |
| $K,B$ | k-nearest 數、beam 寬 | — |
| $z_g,z_g^{\max},z_g^{\min}$ | 地表高（ENU $u$）、走廊內最高／最低地形 | m |
| $z_{lo},z_{hi},z^*,u$ | 高度下界、上界、最低可行剖面、上界剖面 | m |
| $\Delta h_{sep},\delta_{prof},d_{sep},d_{sep,lat}$ | 垂直分層、剖面容差、水平分離、側向分離 | m |
| $d_{turn,c}$ | 迴轉帶端線方向外延（含 Ω 超出量，D38） | m |
| $S_{acc},S^{app}$ | 起飛加速段末狀態（進入路徑起點）、降落減速段起點狀態（返航路徑終點） | — |
| $idx,k$ | 條帶掃向索引 $(dir+1)/2$、轉場型 $2\,idx_l+idx_{l'}$（§8.2） | — |
| $E_{ref}$ | 基準測試電池容量 `reference_battery_wh`（D39） | Wh |

---

## 3 平台模型（表 1 每個鍵）

### 3.1 表 1 鍵 → 模型用途（值不得更改；來源 T1）

| 鍵 | A | B | 用途 |
|---|---|---|---|
| platform_type | lift_and_cruise_vtol_4plus1 | 同 | 狀態機 §3.4；禁止懸停轉向 |
| wingspan_m | 3.20 | 2.18 | 站點檢查 $r_{site}\ge$ 翼展/2 + 2 m（A 3.6 ≤ 8 ✓、B 3.1 ≤ 6 ✓） |
| fuselage_length_m | 1.68 | 1.14 | 僅報告（不入模型） |
| wing_area_m2 | 1.00 | 0.53 | 翼載 $m g/S$ 僅報告 |
| mtow_kg | 24.9 | 7.0 | 硬檢查 $m_i\le$ mtow |
| reference_auw_kg | 24.6 | 6.65 | $m_{ref}$（功率、爬升角、失速修正基準） |
| reference_payload_kg | 5.0 | 0.6 | 空重推估 $m_{empty}=m_{ref}-m_{payload,ref}$（當架次未給 payload_kg 時 $m_{payload}=m_i-m_{empty}$，並警告 `payload_estimated`） |
| payload_max_kg | 5.0 | 1.0（「1.0 以下」；provisional） | 酬載檢查（§6.2 步 6）：A $m_{payload}\le5.0$；B $m_{payload}<1.0$，超過 NO_GO |
| cruise_airspeed_mps | 21.5 | 19.0 | $V_{a,c}$ |
| cruise_airspeed_kmh | 77.4 | 68.4 | 顯示；建構時 assert $=3.6V_a$ |
| min_mission_airspeed_mps | 18.0 | 15.0–16.0 → 規劃 16.0 | $V_{a,\min}$（D9） |
| transition_airspeed_mps | 16.0 | 14.0 | 轉換段指令（未驗證 +2.0）、$d_{tr}$ 推估 |
| stall_airspeed_mps | 14.0 | 12.0 | 失速裕度檢查（D9、D21） |
| max_airspeed_mps | 35.0 | None（待實機確認） | 僅 $V_a^*$ 上限檢查（D10） |
| max_climb_angle_deg | 3.5 | 3.5 | 驗證上限（sim.py），不規劃 |
| planning_climb_angle_deg | 3.0 | 3.0 | $\gamma_c$ |
| max_descent_angle_deg | 5.0 | 5.0 | 驗證上限 |
| planning_descent_angle_deg | 4.0 | 4.0 | $\gamma_d$ |
| max_fw_climb_rate_mps | 1.13 | 1.00 | 建構 assert $V_a\sin3°$（1.125/0.994，5% 內） |
| max_fw_descent_rate_mps | 1.50 | 1.33 | assert $V_a\sin4°$（1.500/1.325） |
| max_bank_angle_deg | 20 | 25 | assert $V_a^2/(g\tan\phi)$≈$R_{theo}$（129.5/78.9）；弧段爬升折減 $\cos\phi_{\max}$ |
| turn_radius_theoretical_m | 130 | 75 | $R_{eff}$ 風放大基準 |
| turn_radius_planning_m | 200 | 120 | $R_{plan}$ |
| vtol_site_clear_radius_m | 8 | 6 | 站點檢查（provisional） |
| obstacle_horizontal_buffer_m | 30 | 25 | 站點檢查（provisional；D13） |
| transition_start_agl_m | 60 | 50 | $H_{ts}$（provisional） |
| transition_complete_agl_m | 80–100 | 70–90 | $H_{tc}$ 區間（provisional；D29） |
| min_fw_operation_agl_m | 120 | 100 | $h_{\min fw}$；D12；任務層可經 `platform_overrides` **只允許調高**（§3.5） |
| wind_resistance_public | Beaufort 5 | 5 | 僅警告門檻：`BEAUFORT_UPPER_MPS[5]=10.7`（6 → 13.8）；PRECHECK 對 `gust_mps > beaufort_upper` 警告 |
| reference_endurance_min | 176 | 136 | $T_{ref}$ → $P_c^{ref}=f_{dep}E_{ref}/T_{ref}$（D39） |
| reference_total_range_km | 242 | 144 | 距離自洽檢查；$R_{ref}/T_{ref}$ 隱含地速對 $V_{a,c}$ 偏差 >10% 時警告 `endurance_range_inconsistent` |
| reserve_fraction | 0.30 | 0.30 | $r$ |
| usable_range_fraction | 0.55–0.60 | 同 | 建構 assert `usable_distance_km ≈ fraction × reference_total_range_km`（±1 km；0.55×242=133、0.60×242=145、0.55×144=79、0.60×144=86） |
| usable_distance_km | 133–145 | 79–86 | 獨立上限 $D^{use}$（未驗證取下緣） |
| symmetric_mission_radius_km | 66–72 | 39–43 | H2 半徑（取下緣）；assert `≈ usable_distance_km/2`（±1 km） |
| airspeed_sensor_validated | false | false | D8 |
| battery_monitor_validated | false | false | $k_{bm}=1.15$、$D^{use}$ 下緣、遙測 SoC 只作趨勢 |
| c2_link_range_km | 待填入 | 待填入 | 必填任務輸入（D11），**逐構型** $P_{link,c}$：`C2Link.range_km` 為單值或 `{A:…,B:…}`；解析順序 mission 值 → spec 覆寫槽 → 皆 None 則 NO_GO 並列鍵名 |
| terrain_clearance_m | 待任務定義 | 同 | 必填任務輸入（D11）；解析順序 `Airspace.terrain_clearance_m` → spec 覆寫槽 → NO_GO |
| alternate_landing_sites_required | true | true | 無備降點 → NO-GO |

PlatformSpec 上的 `c2_link_range_km`、`terrain_clearance_m` 兩欄位只是「承載已實測值的覆寫槽」，預設 None。

### 3.2 運動學與風三角（論文 Eq.1 之擴展）

$$\dot x=V_g\sin\theta,\quad\dot y=V_g\cos\theta,\quad\dot z=V_a\sin\gamma,\quad|\dot\theta|\le V_g/R_{eff}$$

風三角（$W_\parallel=\vec w\cdot\hat t$，$W_\perp=\vec w\cdot\hat n$）：

$$V_g(\theta,V_a)=\sqrt{V_a^2-W_\perp^2}+W_\parallel,\qquad\psi_{hdg}=\theta-\arcsin(W_\perp/V_a)$$

可行條件 $|W_\perp|<V_a$ 且 $V_g\ge V_{g,\min}$（`cfg.v_g_min_mps`，預設 3 m/s）。逆風航向 $\theta=\beta$ 時 $W_\parallel=-W$。

**分層風下的 $R_{eff}$ 取風速**：$W_{R,c}=\max\{W(z):z\in[z_{site}+H_{tc},\ \max(z_{op,c},z_{transit,c})]\ \text{over cells}\}$（保守；`cfg.r_eff_wind_policy="max_layer"`），或 `"op_layer"` 取各胞作業層風速；CostCache 記錄所用 $W_{R,c}$，§12.6 的「$R_{eff}$ 變化 >5%」以此比較。

**轉彎半徑（D1）**：$R_{theo}=V_a^2/(g\tan\phi_{\max})$（A 129.5 ≈ 130、B 78.9 ≈ 75）。空氣圓在風中的地面軌跡為擺線（trochoid），順風相位曲率半徑最大 $=R(1+W/V_a)^2$。

$$R_{eff,c}(W)=\max\Big\{R_{plan,c},\;R_{theo,c}\big(1+W/V_{a,c}\big)^2\Big\},\qquad W^*_c=V_{a,c}\big(\sqrt{R_{plan,c}/R_{theo,c}}-1\big)$$

$W^*$ = $R_{plan}$ 內建裕度可吸收的風速：A 5.17 m/s、B 5.03 m/s；$W=8$ m/s：A 245 m、B 152 m。以地面半徑 $\ge R_{eff}$ 生成的航跡在任何相位需求傾角 $\le\phi_{\max}$（可飛性 §8.8）。

**地面坡度**：$\tan\gamma^g_c(\theta)=\dfrac{V_a\sin\gamma_c\,\kappa}{V_g(\theta)}$，$\tan\gamma^g_d(\theta)=\dfrac{V_a\sin\gamma_d}{V_g(\theta)}$；弧段 $\kappa=\cos\phi_{\max}$（傾角下升力需求），直線 $\kappa=1$。保守方向：爬升距離用順風（$V_g$ 最大）；下降距離同樣用順風最大 $V_g$（實際逆風時可更陡，不利用）。A、$W=8$ 順風：$\tan\gamma_c^g=0.0381$（2.18°）。

**AUW 修正（D21）**：

$$\tan\gamma_c(m)=\tan\gamma_c^{plan}\cdot\mathrm{clip}\big(1-k_\gamma(m/m_{ref}-1),\,0.6,\,1\big),\ k_\gamma=2.0;\qquad V_{s}(m)=V_s\sqrt{m/m_{ref}};\qquad \text{require } V_{a,\min}\ge1.2V_s(m)$$

A @ MTOW 24.9：$\gamma_c=2.93°$，$1.2V_s=16.9\le18$ ✓；B @ 7.0：$2.68°$，$14.8\le16$ ✓。$m>m_{ref}$ 時輸出 warning。轉彎半徑不隨 AUW 變（固定傾角），但 $V_a^*$ 下限隨 $V_s(m)$。

### 3.3 派生量（PlatformSpec 方法）

`R_eff(W)`、`W_star()`、`tan_gamma_c_ground(theta, W, arc)`、`climb_dist(dh, theta, W)`＝$dh/\tan\gamma^g_c$、`descent_dist`、`orbit_gain(W)`（§11.6 積分）、`gamma_c_auw(m)`、`stall_margin_ok(m)`、`transition_cmd()`（D8/D29）。建構時 assert：`cruise_airspeed_kmh==3.6*mps`（±0.1）、$V_a\sin3°$ vs `max_fw_climb_rate_mps`（±5%）、$V_a\sin4°$ vs descent rate、$V_a^2/(g\tan\phi)$ vs `turn_radius_theoretical_m`（±10%）。

### 3.4 飛行模式狀態機

```
HOVER_CLIMB(v_z_up, P_h) → TRANSITION_OUT(逆風, H_ts→H_tc 旋翼輔助, P_tr, t_tr_out) → FW_ACCEL(V_tr,cmd→V_a,c, 直線 d_acc)
  → FW_CLIMB(γ≤γ_c(m), 必要時 orbit R≥R_eff) → FW_CRUISE / FW_OPER(V_a*) → FW_DESCENT(γ≤4°)
  → FW_DECEL(V_a,c→V_tr,cmd, 等高 H_tc, 直線 d_acc) → TRANSITION_IN(H_tc→H_ts 旋翼輔助, P_tr, t_tr_in) → HOVER_DESCENT(v_z_dn, P_h)
  任一 FW 態可 → FW_ORBIT(hold) / MISSED_APPROACH
```

轉換段限制（D7/D8/D37）：站點平均風 $\le W_{go}$、陣風 $\le G_{site}$（未驗證 8、驗證 10 m/s）、側風分量 $|W\sin(\beta-\theta_{tr})|\le X_{\max,c}$（未驗證 6/5、驗證 8/6 m/s）；轉換段航向一律逆風 $\theta_{tr}=\beta$（$W<2$ m/s 時取障礙最少方位）。

**轉換段幾何（D37）**：轉換水平距離 $d_{tr}=V_{tr,cmd}^2/(2a_{tr})+V_{tr,cmd}t_{hold}$（$a_{tr}=0.5$ m/s²、$t_{hold}=8$ s，provisional；驗證後 $V_{tr}$=16/14 → 384/308 m，未驗證 $V_{tr,cmd}$=18/16 → **468/384 m**）；`transition_ground_run_m` 改為**可選覆寫**（None → 公式）。轉換段為旋翼輔助飛行（升力旋翼持續運轉、功率 $P_{tr}=P_h+P_c$），$H_{ts}\to H_{tc}$（起飛）／$H_{tc}\to H_{ts}$（降落）之高度變化由旋翼升力承擔，其坡度 $\arctan\frac{H_{tc,hi}-H_{ts}}{d_{tr}}$（A 4.9°、B 5.9°；表列 400/300 m 時 5.7°/7.6°）**不受固定翼 3°/4° 限制**，但須 $\le\gamma_{tr,\max}$（provisional 8°）；固定翼坡度限制自 $P_{tc}$ 起（起飛）／至 $P_{tc}$ 止（降落）生效。降落側 TRANSITION_IN 同時減速 $V_{tr,cmd}\to$ 懸停。若 `cfg.transition_profile="fw_slope"`（旋翼不輔助），$d_{tr}\leftarrow\max(d_{tr},(H_{tc,hi}-H_{ts})/\tan\gamma_{plan})$（A 爬升 763 m、下降 572 m）並標 provisional。

**加減速段**：FW_ACCEL 自 $P_{tc}$ 沿 $\theta_{tr}$ 直線 $d_{acc}=(V_{a,c}^2-V_{tr,cmd}^2)/(2a_{tr})$、歷時 $t_{acc}=(V_{a,c}-V_{tr,cmd})/a_{tr}$（A 138 m／7 s、B 105 m／6 s），功率 $p_{accel}P_c$（$p_{accel}=1.2$，provisional）；其終點 $S_{acc}=(P_{tc}+d_{acc}\hat t(\theta_{tr}),\theta_{tr},R_{eff})$ 為進入路徑起始狀態（§8.2、§10.1）。FW_DECEL 對稱：等高 $H_{tc}$、終點 $P_{tc}$，起點 $S^{app}=(P_{tc}-d_{acc}\hat t(\theta_{app}),\theta_{app},R_{eff})$ 為返航路徑目標狀態（§8.7）。

### 3.5 補充參數（表 1 之外，皆 provisional，可覆寫）

`battery_nominal_wh`（缺省 → 基準分鐘模式 §5.7；1500/420 Wh 只是示例值）、`reference_battery_wh`（基準測試電池，D39；缺省 = `battery_nominal_wh` 並警告 `reference_battery_assumed`）、`reference_test_depth_of_discharge` $f_{dep}$=1.0、`hover_power_ratio`=4.0、`t_tr_out/in`=20/25 s、`a_tr`=0.5 m/s²、`t_hold`=8 s、`transition_ground_run_m`=None（可選覆寫 $d_{tr}$）、`gamma_tr_max_deg`=8、`p_accel_ratio`=1.2、`vtol_climb_rate_mps`=2.0、`vtol_descent_rate_mps`=1.0（最後 10 m 0.5）、`prop_efficiency`=0.6、`idle_descent_ratio`=0.3、`t_hover_reserve_s`=15、`dV_tr_margin`=2.0、`k_gamma_auw`=2.0、`crosswind_site_max`=(6,8)/(5,6)、`gust_site_max`=(8,10)、HERO `payload_max_kg`=1.0（「1.0 以下」之改寫）。

以上鍵構成 `PlatformSpec.provisional_supplementary`；與表 1 的 `provisional` 集合一併由 PRECHECK 寫入 `warnings["provisional_params"]`。

**任務層覆寫**：`MissionInputs.platform_overrides: dict[cls, dict[key, value]]`，`build_context` 以 `dataclasses.replace` 套用；只允許覆寫 `provisional ∪ provisional_supplementary` 與 `min_fw_operation_agl_m`（只允許調高），其餘鍵拒絕並丟 `MissionInputError`；覆寫後自 provisional 集合移除該鍵並記錄 `overridden(key, old→new)`。

---

## 4 感測器與覆蓋幾何

### 4.1 LiDAR（A）

可用掃描角 $\theta_{use}$（預設 70°）、點率 $f_p$、有效回波比 $\eta_{ret}$、需求點密度 $\rho_{req}$、最大測距 $r_{\max}$、測距裕度 $k_{rng}$（0.9）：

$$w_A(h)=2h\tan\tfrac{\theta_{use}}{2},\quad s_A=w_A(h_{\min,l})(1-o_{side,A}),\quad V_g^{cap,A}=\frac{f_p\eta_{ret}}{w_A(h)\rho_{req}},\quad h_{\max,A}=k_{rng}\,r_{\max}\cos\tfrac{\theta_{use}}{2}$$

例：$h=150$：$w_A=210.1$ m，$s_A=157.5$ m；$h=160$：$w_A=224.1$、$s_A=168.0$ m；$f_p=600$ kHz、$\eta_{ret}=0.5$、$\rho_{req}=20$：$V^{cap}=71$ m/s（不綁）；$r_{\max}=450$：$h_{\max}=332$ m。

**硬體與需求分離（D43）**：$\theta_{use},f_p,r_{\max}$ 屬 `SensorHardware`（機上硬體）；$\rho_{req},o_{side},h^{target}$ 屬 `PayloadRequirement`（區域需求，`AreaSpec.payload_req_id` 指向）。本節與 §4.2 的硬體數值（70°、600 kHz、450 m；16 mm、23.5×15.6 mm、3.9 µm、1 Hz）**皆為示例值（表 1／表 2 未提供）**，`is_default_example=True`；使用示例硬體時 PRECHECK 警告 `sensor_spec_default_example(cls)`。

### 4.2 相機（B）

焦距 $f$、感光元件 $S_w\times S_h$、像元 $p$、快門率 $f_s$、需求 $GSD_{req}$：

$$w_B(h)=h\frac{S_w}{f},\ \ l_B(h)=h\frac{S_h}{f},\ \ GSD(h)=h\frac{p}{f},\ \ h_{\max,B}=\frac{GSD_{req}f}{p},\ \ s_B=w_B(h_{\min,l})(1-o_{side,B}),\ \ V_g^{cap,B}=l_B(h)(1-o_{fwd})f_s$$

例：$f=16$ mm、$23.5\times15.6$ mm、$p=3.9$ µm、$f_s=1$ Hz、$GSD_{req}=5$ cm：$h_{\max,B}=205$ m；$h=100$：$w_B=146.9$、$s_B=102.8$、$l_B=97.5$ m、GSD 2.44 cm、$V^{cap}=29.3$ m/s。

**GSD 可達性檢查**：需 $h_{\max,B}(GSD_{req})\ge h_{\min fw,B}+\Delta h_{band,\min}$（示例硬體：$GSD_{req}\ge(100+20)\cdot3.9\text{ µm}/16\text{ mm}=2.93$ cm）；否則該 (區, B) 標 `infeasible(sensor: gsd_unattainable)` 而非被 clip 靜默抬到 $h_{\min fw}$。同式用於 §12.7 之 $GSD_{conf}$。

### 4.3 論文梯形 FOV（選項 `fov_model="trapezoid"`）

論文 Eq.(2)(3) 以本規格明寫（PDF 抽字後 Eq.(2) 的分數線遺失；只有**除以** $\sin$ 的斜距投影形式能重現論文數值）：

$$w_b=\frac{2h\tan(\alpha_h/2)}{\sin(\alpha_m-\gamma+\alpha_v/2)},\quad w_f=\frac{2h\tan(\alpha_h/2)}{\sin(\alpha_m-\gamma-\alpha_v/2)},\quad l_f=\frac{h}{\tan(\alpha_m-\gamma-\alpha_v/2)}-\frac{h}{\tan(\alpha_m-\gamma+\alpha_v/2)},\quad d_f=\frac{0.5h}{\tan(\alpha_m-\gamma-\alpha_v/2)}+\frac{0.5h}{\tan(\alpha_m-\gamma+\alpha_v/2)}$$
$$w=(w_b+w_f)/2,\qquad l=d_f+l_f/2$$

角度須在 $(0°,180°)$；$\alpha_m+\alpha_v/2>90°$ 時後緣越過天底、$\tan$ 為負（允許）。測試值 $h=1200,\alpha_m=72.7°,\alpha_v=45°,\alpha_h=76.98°$ → $w=2200.1$、$l=999.8$ m（乘 $\sin$ 的誤讀給 1683 m）。

### 4.4 速度選擇（每航段）

$$V_a^*(\theta)=\max\{V_a\in[V_{a,\min}(m),V_{a,c}]:\ V_g(\theta,V_a)\le V_g^{cap}\}\quad(\text{作業段})$$

若 $V_g(\theta,V_{a,\min})>V_g^{cap}$ → 該方向不可行（$\vec t^{oper}$ 該分量 $=\infty$）。**單向可行的胞仍可覆蓋（D34）**：IDP 以同向轉場（racetrack，§8.2 第三／四型：掃完後在胞外回飛再同向掃下一條）串接，方向交替不再是硬限制；§7.3 的 $T_{proxy}$ 對單向可行胞改用「單向作業時間 + 回飛時間」。兩向皆不可行 → 提高 $h_{op}$（若 GSD 允許）或降低 $o_{fwd}$ 或改方向；仍不可行 → H5 失敗。轉移／進入／返航段用 $V_{a,c}$。`max_airspeed_mps` 只檢查 $V_a^*\le$ 上限。

### 4.5 條帶寬用保守 AGL、端點外延

足跡寬永遠以 $h_{\min,l}$（該條帶走廊內最高地形之上的最小 AGL，§7.4／§11.3）代入 $w_c(\cdot)$；$w_c$ 對 $h$ 單調遞增 ⇒ 沿線任一點實際足跡 $\ge w_c(h_{\min,l})$。相機另檢查 $AGL_{\max,l}\le h_{\max,B}k_{GSD}$（$k_{GSD}=1.0$，§11.5 可放寬至 1.2 並記錄）。端點外延 $e_{ext,c}=\max(V_{g,\max}t_{settle},\ l_B(1-o_{fwd}))$，$t_{settle}=2$ s（滾轉安定；相機第一張影像前向重疊）。

### 4.6 作業高度與高度帶政策（D12）

每胞、每構型：
$$h_{op,c}=\mathrm{clip}\big(h^{target}_c,\ h_{\min fw,c},\ \min(h_{\max,c},h_{ceil,c})\big),\qquad h_{ceil,c}=\texttt{waiver\_max\_agl\_m}\ \text{if given else}\ \texttt{max\_agl\_m}$$
高度帶寬（**胞多邊形層級**，避免「條帶寬依賴 $h_{op}$、$h_{op}$ 依賴條帶走廊」的循環）：以胞多邊形 $\oplus$ 迴轉帶（D38）為範圍 $\Omega_q$，
$$\Delta h_{band}=\big(\min_{\Omega_q}z_g+h_{ceil}\big)-\big(\max_{\Omega_q}z_g+\max(h_{\min fw},h_{clr})\big)$$
（較條帶走廊定義更保守、無循環）。可指派條件 $\Delta h_{band}\ge\Delta h_{band,\min}=20$ m（坡度包絡在起伏地形上需要的活動空間）；條帶層級的可行性由 §7.4 的 `LOCAL_ENVELOPE.feasible` 回報。Pioneer 於 $h_{ceil}=120$ 且無豁免 → $\Delta h_{band}\le0$ → 該胞對 A 不可行（PRECHECK warning `A_needs_waiver`），改由 B 或要求 `waiver_max_agl_m`。LiDAR 目標高 $h^{target}_A$ 預設 150 m（足跡／密度折衷，可設）；相機 $h^{target}_B$ 預設 $\min\big(h_{\max,B}/k_{GSD,relax},\ h_{\max,B}-\Delta h_{band,\min}\big)$（示例硬體 $\approx171$ m）——不取 $h_{\max,B}$ 本身，否則 $z_{des}=z_g^{\max}+h_{\max}\ge z_{hi}=z_g^{\min}+h_{\max}$ 在任何橫向起伏下都貼著上界飛、微小地形特徵即觸發 INFEASIBLE(OPER)。另需 $h_{\max,B}\ge h_{\min fw,B}+\Delta h_{band,\min}$（§4.2 GSD 可達性）。

---

## 5 能量模型與可行性（含 RTB、備降點）

### 5.1 功率（由表 1 建立；D6/D20）

$$P_c^{ref}=\frac{f_{dep}E_{ref,c}}{T_{ref,c}},\quad P_c(m)=P_c^{ref}\Big(\frac{m}{m_{ref}}\Big)^{1.5},\quad P_h(m)=k_hP_c(m),\quad P_{tr}=P_h+P_c,\quad P_{acc}=p_{accel}P_c$$
$$P_\uparrow(m)=P_c(m)+\frac{mgV_a\sin\gamma}{\eta_p},\qquad P_\downarrow(m)=\max\Big(\eta_{idle}P_c(m),\ P_c(m)-\frac{mgV_a\sin|\gamma|}{\eta_p}\Big)$$

$E_{ref,c}$=`reference_battery_wh`（**執行 176/136 min 基準測試那顆電池**的容量，D39；不是當架次標稱容量——否則換 1200 Wh 電池 $P_c^{ref}$ 低估 20%、1800 Wh 高估 20%，與 $k_{unc}$ 同量級且方向不定）。$f_{dep}$=`reference_test_depth_of_discharge`=1.0（provisional：假設基準測試耗盡整顆電池；測試留有保留量時 $P_c^{ref}$ 偏高＝保守）、$k_h=4.0$、$\eta_p=0.6$、$\eta_{idle}=0.3$、$p_{accel}=1.2$。例（$E_{ref,A}=1500$ Wh）：$P_c^{ref}=1500/(176/60)=511$ W、$P_h=2045$ W；A 爬升額外功率 $24.6\cdot9.81\cdot1.125/0.6=453$ W；B（420 Wh）$P_c^{ref}=185$ W、$P_h=741$ W、爬升額外 108 W。

自洽：$R_{ref}/T_{ref}$ = 22.9 m/s（A）、17.6 m/s（B），與 $V_a$ 差 +6%/−7%；PRECHECK 於 `derived` 回報 $P_c^{ref}$ 與此偏差，$|{\rm 偏差}|>10\%$ 時警告 `endurance_range_inconsistent`。能量以 $T_{ref}$ 為準，距離自洽以 $\min(V_aT_{ref},R_{ref})$。單元測試 `pack_size_invariant_power`：換電池組容量不改變 $P_c^{ref}$。

### 5.2 段能量、不確定性、溫度

$$e_{seg}=P_{mode}(m)\,t_{seg}\,k_{unc},\qquad k_{unc}=k_{bm}k_{as},\ k_{bm}=1.15\ (\texttt{battery\_monitor\_validated=false}),\ k_{as}=1.05\ (\texttt{airspeed\_sensor\_validated=false})$$
驗證後對應係數為 1.0：$k_{bm}=$`cfg.k_bm` 若 `battery_monitor_validated=false` 否則 1.0，$k_{as}$ 同理（`EnergyModel.k_unc` 由 spec 旗標 × cfg 取值，可覆寫）。溫度容量係數 $\eta_T=1-c_T\max(0,T_0-T_{bat})$（`temp_cap_coef_per_c`=0.01/°C、$T_0=10$ °C、下限 `temp_cap_floor`=0.8），$T_{bat}$=`SortieEnergy.battery_temp_c`（None → 取 `WeatherObs.temp_c` 並警告 `battery_temp_assumed`）；`WeatherObs.temp_c` 只用於放飛／中止門檻。

### 5.3 起降固定成本與可規劃能量（保留只扣一次）

$$e^{TO}=k_{unc}\Big[P_h\frac{H_{ts}}{v_{z,up}}+P_{tr}t^{out}_{tr}+P_{acc}t_{acc}\Big],\qquad e^{LDG}=k_{unc}\Big[P_{acc}t_{acc}+P_{tr}t^{in}_{tr}+P_h\,t_{dn}+P_ht_{hov,res}\Big],\quad t_{dn}=\frac{H_{ts}-10}{v_{z,dn}}+\frac{10}{0.5}$$
$$E^{use}_i=\eta_TE^{avail}_i-rE_{nom,c}-e^{TO}-e^{LDG}$$
$t_{dn}$ 含「最後 10 m 以 0.5 m/s」（A 70 s、B 60 s），與 §11.7 `seg_v` 共用同一函式 `hover_descent_time(H_ts)`。保留 $rE_{nom}$ 以**當架次標稱容量**計、任務結束後仍持有；本規格中保留只在此扣一次，IDP 與逐點檢查皆以 $E^{rem}$ 對 $rE_{nom}+e^{alt}$ 比較，不再重複扣除（修正草案雙重扣除）。例 A（可用 1350 Wh）：$e^{TO}=39.2$、$e^{LDG}=81.2$、$E^{use}=779.6$ Wh ≈ 75.8 min 巡航當量；B（420/380 Wh）：12.9、26.9、$E^{use}=214.3$ Wh ≈ 57.5 min。

### 5.4 可用距離交叉檢查（表 1 `usable_distance_km`、`symmetric_mission_radius_km`）

計畫巡航等效距離 $D^{plan}_i=\sum_{seg}\frac{e_{seg}}{P_c(m_i)k_{unc}}V_{a,c}$（含 RTB）必須 $\le D^{use}_c$（未驗證取下緣 133/79，驗證取 145/86）。模型自洽（$e^{TO}+e^{LDG}$ 預設值）：驗證後 A：$(1-0.30-0.061)\times176$ min$\times21.5$ = 145 km（表上緣 ✓）；未驗證（$k_{unc}=1.2075$）A ≈ 120 km（≤133，較表保守）、B ≈ 81 km（∈[79,86] ✓）；驗證後 B 模型值 97 km 超過表上緣 86 km（B 基準測試平均地速 17.6 < 19 m/s 所致）→ 由 $D^{use}$ 上限截止，採表值。結論：能量模型為主、$D^{use}$ 為獨立硬上限，兩者取嚴。任一航點距基地投影距離 $\le\rho^{sym}_c$（下緣）為第二交叉檢查（H2）。

### 5.5 備降能量與逐航點檢查（C4）

$$e^{alt}(x,a)=k_{unc}\Big[P_c\frac{|D_{avoid}(x\to S^{app}_a)|}{V_g^{worst}}+P_\downarrow t_\downarrow+e^{LDG}\Big],\quad V_g^{worst}=\max(V_{g,\min},V_a-W_{\max}(z))$$

**EnergyBudgetTable（不再是直線估計）**：建表時對每個（條帶端點 $\times$ 兩個終端狀態、Ω/U 轉場最外點）$\times$ 每個降落點以 AVOID（§8.6，含禁區／keep-out 繞行）＋無釘住 ENVELOPE（§11.2，含地形／稜線所需爬升與 $\Delta h/\tan\gamma^g$ 的爬升水平距離）算**真實時間與能量**（含 $P_\uparrow$、$P_\downarrow$、$k_{unc}$、逆風）並快取（$2n\times|\mathcal A\cup\{H\}|$ 次，一次性，向量化）。任意點 $x$ 的 `point_wh(x, a)` 以上界化：取最近條帶端點 $p^*$ 之表值 $+$ 直線增量 $k_{unc}P_c\|x-p^*\|/V_g^{worst}$ $+$ 爬升項 $k_{unc}(P_\uparrow-P_c)\max(0,z_{ridge}(x\to p^*)+h_{clr}-z_x)/(V_a\sin\gamma_c)$。反例（B 於 250 m 谷地、備降點在 600 m 稜線後）：爬升 350 m 需 12.8 Wh + 6.68 km 水平 29.6 Wh ≈ 42 Wh = $E^{use}_B$ 的 19%，直線估計完全遺漏。備降點須在鏈路覆蓋內且通過站點檢查（§11.8）。

```
ENERGY_SAFE_CHECK(route_i, E_avail_i):                       # 中間點以「段末剩餘 + 段內剩餘」計
  E = eta_T*E_avail_i - e_TO                                 # S_acc（加速段末）的剩餘能量
  for seg in route_i.segments up to and including RTB (止於承諾點 P_tc = FW_DECEL 終點):
      e_seg = seg.energy_wh()                                # = Σ_j P_mode(j)*dt_j*k_unc（逐樣本，含爬升/下降/加速功率）
      E_end = E - e_seg
      for x_j in seg.samples():                              # 每 d_d=25 m 一樣本（弧段 min(d_d, RΔψ)）
          E_j = E_end + e_seg_from(x_j, seg.end)             # 尚未耗掉的段內能量加回
          e_alt = min_{a in ALT ∪ {H}} budget.point_wh(x_j, a)
          if E_j < r*E_nom + e_alt: return INFEASIBLE(seg, x_j, deficit = r*E_nom + e_alt - E_j)
      E = E_end
  # P_tc 之後（TRANSITION_IN、HOVER_DESCENT）只要求 E ≥ r*E_nom：備降可達性止於承諾點
  E_land = E - e_LDG_remaining；assert E_land >= r*E_nom
  # missed approach 另作單獨檢查（§11.7）：
  assert E(P_tc) >= r*E_nom + e_MA，e_MA = e(沿 θ_app 以 γ_c 爬至 z_tc+50) + e(一圈下降盤旋) + e_LDG
  return FEASIBLE, E_land
```

（舊版把逐點備降檢查延伸到觸地樣本，等於要求落地餘量 $\ge rE_{nom}+e^{LDG}+e(\to$ALT$)$≈36.6%，無故拒絕可行計畫；現止於 $P_{tc}$。）

### 5.6 遙測策略

`battery_monitor_validated=false`：遙測 SoC 僅作**趨勢**（§12.3 的 $\delta$ 比對），不得直接作為 $E^{rem}$；驗證後可以遙測值覆寫 $E^{rem}$（限幅 $\pm10\%$）。

### 5.7 基準分鐘模式與可用電量解析

`SortieEnergy` 解析順序：`available_wh` → `available_mah × pack_voltage_nom_v / 1000` → `soc_fraction × nominal`；$E_{nom}$ 取 `SortieEnergy.pack_nominal_wh`（當架次實際電池組標稱）→ `spec.battery_nominal_wh`。給了 Wh／mAh 而兩個 nominal 皆 None → PRECHECK **NO_GO** 並列鍵名（比照 D11，不給靜默預設；因 $rE_{nom}$ 與 $P_c^{ref}$ 都需要標稱值）。使用 §16.2 示例值 1500/420 Wh 時警告 `battery_nominal_example`。

**基準分鐘模式**只允許 `soc_fraction` 輸入：`reference_battery_wh` 與 nominal 皆 None 時能量單位改為「基準巡航分鐘」（$1$ unit $=P_c^{ref}\cdot60$ s，$P_c^{ref}\equiv1/60$ unit/s），$E^{avail}$ = SoC × $T_{ref}$；其餘公式不變。輸出報告標示單位。

---

## 6 任務輸入與前置檢查（表 2 每列）

### 6.1 表 2 → 內部表示

| 表 2 輸入 | 內部表示（`mission.py`） | 用途 |
|---|---|---|
| 起飛點與備降點（WGS84、MSL/AGL） | `Site(lat, lon, alt_msl_m|None, alt_agl_m|None, kind, clear_radius_m, approach_heading_deg)`；MSL／AGL 二擇一（AGL 以 DEM 換算，皆 None 由 DEM 取）；WGS84→ENU | 起降走廊、RTB、備降可行性 |
| 任務區域（Polygon/GeoJSON） | `AreaSpec(polygon_wgs84 (lat,lon), holes, sensor_req, priority, time_window_s, payload_req_id)`；`load_mission` 接受 GeoJSON Feature/Polygon/MultiPolygon：自動交換 (lon,lat)→(lat,lon)、外環→polygon、內環→holes、MultiPolygon 拆成多個 AreaSpec（`area_id` 加尾碼 `_1,_2,…`） | 分解、geofence、條帶 |
| 地形模型 DEM/DSM | `DemGrid(lat0, lon0, dlat, dlon, z_msl, is_dsm, vertical_datum, geoid_undulation_m, sigma_z_m)` | 坡度、跟隨、AGL、淨空（$\sigma_z$ 進入 $h_{clr}$ 下限） |
| 障礙物（點／線／面＋高度） | `Obstacle(geom_type, coords, top_msl_m|height_agl_m, category, keepout_buffer_m)`；頂高於 `KeepoutSet.build` 轉為 ENU $z$（D42） | 燒入地形（D24）＋ keep-out |
| 風場（分層速度／風向） | `WindLayer(z_bottom, z_top, z_ref∈{msl,agl}, speed_mps, dir_from_deg, gust_mps)` 列表；`WindField` 內部以 ENU $z$ 存層界（以 home 位置換算；$50$ km 外曲率差 196 m 記為已知近似） | 地速、耗時、逆風返航、$R_{eff}$（$W_{R,c}$ 政策 §3.2）、陣風 → 轉換與 $R_{eff}$ 裕度（$W+G$ 用於 $R_{eff}$ 上界檢查） |
| 當架次 AUW | `SortieEnergy.auw_kg`、`payload_kg|None`（None → $m_i-(m_{ref}-m_{payload,ref})$ 推估並警告） | $P(m)$、$\gamma_c(m)$、$V_s(m)$、酬載檢查 |
| 可用電量（Wh 或 mAh） | `SortieEnergy(available_wh | available_mah×pack_voltage_nom_v | soc_fraction, pack_nominal_wh, battery_temp_c)`；解析順序與 NO_GO 規則見 §5.7 | $E^{avail}$、$E_{nom}$、$\eta_T$ |
| 酬載任務需求 | 兩層（D43）：`UAVInstance.sensors: (SensorHardware, …)`（主感測器在前；焦距／感光元件／像元／快門上限／FOV／點率／測距）與區域級 `PayloadRequirement(req_id, kind, gsd_required_m|None, density_pt_m2|None, side_overlap, forward_overlap, shutter_rate_hz ≤ 硬體上限, h_target_agl_m|None)`；`sensor_model(hw, req) → Sensor`，區域需求覆寫硬體預設；`DecomposeContext.sensors[(cls, req_id)]` | 航高、速度、間距、端點延伸（逐區生效） |
| C2 鏈路覆蓋 | `C2Link(range_km: float | {A,B}, coverage_polygon, ground_station, lost_link_policy per UAV, t_ll_s)`；$P_{link,c}$ 逐構型 | 半徑硬限制、失鏈策略 |
| 空域與法規 | `Airspace(max_agl_m=None, waiver_max_agl_m, max_msl_m, no_fly: [NoFlyZone(poly, z_floor, z_ceil, window)], time_window_s, terrain_clearance_m)`；`max_agl_m` 與 `max_msl_m` 皆 None → 採 120 m AGL 並警告 `airspace_ceiling_defaulted_120`，`derived` 記錄 $h_{ceil}$ 來源 | 高度帶、禁區、時間窗、$h_{clr}$ |
| 作業氣象門檻 | `WeatherThresholds(go: mean_wind, gust, visibility, precip, temp_range; abort: mean_wind, gust, visibility, precip, temp_range)` + `WeatherObs`（`temp_c` 僅門檻用） | 放飛與中止 |

### 6.2 前置檢查（PRECHECK）

```
PRECHECK(inputs) -> FeasibilityReport:          # 位於 preflight.py（decompose/altitude 之上），mission.py 只含資料型別
  1 必填：每構型 P_link_c（c2.range_km[c] 或 coverage_polygon，其次 spec 覆寫槽）；terrain_clearance_m（Airspace → spec 槽）；
    alternates 非空（若 required）；E_nom 可解析（§5.7）；缺 → NO_GO 並列鍵名
  2 氣象放飛：obs.mean_wind ≤ min(W_go_c ∀c in fleet)；gust ≤ G_go；vis ≥ vis_go；precip ≤ p_go；T ∈ T_range → 否則 NO_GO
    警告：obs.gust_mps > BEAUFORT_UPPER_MPS[spec.wind_resistance_public]（Beaufort 5 → 10.7、6 → 13.8 m/s；gust 為尖峰值非增量）
  3 座標：GeoFrame(H)；所有輸入 → ENU（MSL 高度 → ENU z，D42）；多邊形 make_valid、面積>0；區域須在 DEM 範圍內
  4 地形：TerrainGrid（步長 dem_step_m=10，雙線性）；障礙燒入（D24）；h_clr ≥ 3·sigma_z + 30 若 sigma_z 已知，否則以輸入值並警告
  5 站點（H 與每個備降點）：DEM 坡度 < cfg.site_max_slope_deg(5°)；r_site_c 內無障礙；b_obs_c 內無高於 site+H_ts 之障礙；
    轉換走廊淨空（§11.8，含 P_tc 後 (h_minfw−H_tc)/tan γ_c^g 之 CLIMB 走廊與 P_tc 前之 DESCENT 走廊）；
    站點在鏈路內；站點風（側風、陣風）≤ 轉換限（§3.4）
  6 機體：m_i ≤ mtow；payload_kg ≤ payload_max（A ≤5.0；B <1.0；payload_kg 缺省以 m_i−(m_ref−payload_ref) 推估並警告 payload_estimated）；
    E_use_i > 0；stall_margin_ok(m_i)；V_a,min(m) ≤ V_a,c
  7 鏈路／半徑裁切：Z_k ← Z_k ∩ (P_link_c ⊖ d_turn,c) ∩ Disc(H, ρ_sym_c − d_turn,c) \ ∪(NoFly(有效窗) ⊕ b_trk)（每構型；先內縮迴轉帶 D38 再裁）；
    裁掉部分記錄 UNCOVERABLE(reason)
  8 高度帶（§4.6，胞多邊形層級）：對每區、每構型算 Δh_band；<20 m → 該 (區,構型) 不可行（A 於 h_ceil=120 無豁免 → warning A_needs_waiver）；
    相機另檢查 h_max,B(GSD_req) ≥ h_minfw,B + Δh_band,min（否則 infeasible(sensor: gsd_unattainable)）
  9 感測可行：對每 (k,c)：κ(σ_k,c)=1 且高度帶可行且 ∃θ 使 V_a* 存在；每區至少一構型可行，否則 UNCOVERABLE(sensor)
 10 時間窗：airspace.time_window 內能容納 min 估計 makespan（粗估）；區域窗與空域窗交集非空
 11 未驗證航電：V_tr += dV_tr_margin；H_tc_cmd = hi；k_as=cfg.k_as；k_bm=cfg.k_bm；D_use 取下緣
 12 警告彙整：provisional ∪ provisional_supplementary 仍在使用之鍵、reference_battery_assumed、battery_nominal_example、
    sensor_spec_default_example(cls)、airspace_ceiling_defaulted_120、endurance_range_inconsistent、overridden(key)
 13 回傳 FeasibilityReport(status ∈ {GO, GO_WITH_WARNINGS, NO_GO}, errors[], warnings[], per_area{}, per_uav{}, derived{R_eff, V_g^cap, h_op, w, s, P_c_ref, h_ceil_source})
```

感測相容 $\kappa(\sigma,c)$ 以機上 `available_kinds`（由 `UAVInstance.sensors` 推導，不再有 `a_has_rgb` 布林欄位）判定：lidar→需 `lidar`；rgb→需 `rgb`（B 原生；A 掛 RGB 時亦可）；both→需 A 與 B 各一套，或同一架 A 同時掛 LiDAR 與 RGB 時 A 可獨立完成（此時 A 的兩套條帶各用對應 `Sensor` 的寬度：分兩套鋪設，或取較窄者鋪一套）；any→皆可。

---

## 7 區域分解

### 7.1 總流程

```
DECOMPOSE(Z_k, σ_k, req_k, params) -> cells:
  P_area = simplify_outward(Z_k, δ_edge, ε_dp)             # §7.2, D31：Z_k ⊆ P_area
  F      = P_area \ ∪(NoFly_active ⊕ disc(b_trk))          # D41：禁區先 ⊕ 追蹤半寬再扣除；障礙物不扣（作業段以高度飛越）
  for P in components(F):                                   # shapely 差集可能為 MultiPolygon：逐一處理
    Q = convex_partition(P, τ_merge)                        # §7.2，含洞
    for Q_q in Q:
      Q_q ← (Q_q ⊕ disc(δ_edge)) ∩ F                       # 胞只在區域內部邊界互相重疊 2δ_edge；禁區邊界不外擴（D41）
      for c in feasible_types(σ_k, Q_q):                    # §4.6 高度帶（胞層級）、§6.2 感測；sensor = sensors[(c, req_k)]
          C = direction_candidates(Q_q, c)                  # §7.3（含 β、β+π）
          for θ in C:  T_proxy[c,θ] = proxy_time(Q_q, θ, c) # §7.3（快速；含迴轉帶 D38 禁區檢查）
          for θ in top3(T_proxy[c,·]):                       # 精算前三名
              strips[c,θ] = lay_strips_conservative(Q_q, θ, c)   # §7.4（含局部包絡回饋）
              drop strips whose (p^L↔p^R ⊕ b_trk) ∩ NoFly_active ≠ ∅ → 該方向不可行（D41，含 e_ext 外延）
              T[c,θ]      = exact_time(strips[c,θ], θ, c)
          θ*_c = argmin_θ T[c,θ]；cell.strips[c] = strips[c,θ*_c]；cell.alt[c] = 前 3 名（配置階段可換向）
      cell.fit[c] = suitability(Q_q, c)                      # §7.6
  residual_candidates(cells)                                 # §7.5（僅 σ=any）：產生 (A 子胞, B 殘帶) 候選對，k_A 由 §9.4 拍賣決定
  return cells
```

### 7.2 簡化與凸分割（含洞）

**只外擴的 DP 簡化（D31）**：$P=\mathrm{simplify}(Z\oplus\mathrm{disc}(\delta_{edge}),\varepsilon)$，$\varepsilon=0.1\min_cs_c$、$\delta_{edge}=k_\sigma\sigma_{xy}+e_{xt}+\delta_w/2$（預設 15 m；$\sigma_{xy}$ 定位誤差、$e_{xt}$ 追蹤誤差、$\delta_w=w\sigma_h/h$ DEM 誤差引起之寬度誤差）。DP 保證原邊界每點距簡化邊界 $\le\varepsilon<\delta_{edge}$ ⇒ $Z\subseteq P$。洞：洞邊界向內縮（$\ominus\delta_{edge}$）後同樣簡化。

**凸分割**（不整體凸化；反射頂點切割，論文引 [42] 思路）：

```
CONVEX_PARTITION(P, τ_merge, A_min = s_ref²):
  if P is MultiPolygon: return ∪ CONVEX_PARTITION(p) for p in P.geoms
  if no holes and (is_convex(P.exterior, tol=2°) or concavity(P) ≤ τ_merge): return [P]   # 近凸即接受（凹度 ≤ 半個足跡的凹口由端點延伸覆蓋）
  V_r = reflex_vertices(P) ∪ all hole vertices               # 洞頂點一律視為反射
  best = None
  for v in V_r:
    for line in {extend(prev_edge(v)), extend(next_edge(v)), bisector(v)} ∪ {v→u : u ∈ vertices visible from v}:
      parts = shapely.split(P, line);  if any(area(p) < A_min) continue
      score = Σ_p n_strips_est(p) + λ_cut·len(line∩P) + λ_n·(len(parts)−1)   # n_strips_est = ⌈min_width(p)/s_ref⌉
      best = argmin score
  if best is None:
      if A_min > 0: return CONVEX_PARTITION(P, τ_merge, A_min = 0)   # 小凹口：允許碎片，交由 MERGE 吸收
      else: return [P] with flag near_convex_forced                  # 保證有回傳；§7.4 端點取帶內極值不依賴凸性
  cells = ∪_p CONVEX_PARTITION(p, τ_merge, A_min) for p in best.parts
  MERGE: 相鄰 (Qa,Qb) 若 concavity(Qa∪Qb) ≤ τ_merge = 0.5·min_c w_c 且合併不增加最佳方向條帶數 → 合併（Lien–Amato 準則；
         深度小於半個足跡的凹口由 Eq.(8)(9) 之「帶內頂點延伸端點」自然覆蓋）
  return cells
```

反例（修正前）：500×500 m 方形角落 20×20 m 凹口，$A_{\min}=s_{ref}^2$ 使所有切線被拒、`best=None` 無回傳；現由第一行「凹度 20 m ≤ τ_merge」直接接受。

備選：Hertel–Mehlhorn（耳切三角化＋去除非必要對角線，洞先橋接）＋同一 MERGE。近凸胞（凹度 ≤ τ）下 §7.4 端點取「帶內裁剪子多邊形之 $u$ 極值」，與凸性無關，仍正確。

### 7.3 候選掃描方向與風／地形感知評分（D15）

候選 $C_q=\{$各邊方位$\}\cup\{\theta_{minw}\}\cup\{\beta,\beta+\pi\}\cup\{\beta\pm90°\}\cup\{$等高線主方向 = DEM 梯度主軸 ⊥$\}$（順逆風方向成本極低，明列以免窄長胞長軸與風向一致時只評到側風候選）。$\theta_{minw}$：論文最小寬度方向（對每邊求最遠頂點距離，$w_{q,\min}=\min_m\max_udis_{mu}$）。

$$T_{proxy}(\theta,c)=\sum_l\frac{|L_l|}{2}\Big(\frac{1}{V_g^*(\theta)}+\frac{1}{V_g^*(\theta+\pi)}\Big)+(n_l-1)\,t_{turn}(s_c,R_{eff,c})+\text{pen}_{slope}(\theta,c)+\text{pen}_{\Delta z}(\theta,c)+\text{pen}_{keepout}(\theta,c)$$

- $n_l=\lceil(W(\theta)-(w_c-s_c))/s_c\rceil$，$W(\theta)$ 為垂直於 $\theta$ 的投影寬。
- $t_{turn}=\min\{L_\Omega(s),\ L_U(ks)\}\cdot\frac{1+(W/V_a)^2}{V_a}$，$k=\lceil2R_{eff}/s\rceil$，$L_\Omega=R(\pi+4\arccos\frac{s+2R}{4R})$，$L_U=\pi R+(ks-2R)$（D5/D14；修正草案 $\pi R/V_a$ 低估 2 倍）。
- **單向可行胞**（$V_g(\theta)>V_g^{cap}$ 或 $V_g(\theta+\pi)>V_g^{cap}$，D34）：第一項改為 $\sum_l|L_l|/V_g^*(\theta_{ok})$，$t_{turn}$ 改為同向轉場代理 $t_{rt}=(\pi R_{eff}+|L_l|)\cdot\frac{1+(W/V_a)^2}{V_a}$ 之回飛（不再取兩向平均，否則 $T_{proxy}=\infty$ 誤判方向不可行）。
- $\text{pen}_{slope}=k_s\sum_l\max\big(0,\overline{|\partial_sz_g|}_l-\tan\gamma^g_c\big)\cdot|L_l|/V_g$，$k_s=20$（有限；沿等高線偏好內建）；若 $\max$ 沿線坡度 $>\tan\gamma_c$ 加旗標 `slope_exceeds`，並在 §11.5 處置。
- $\text{pen}_{\Delta z}=\sum_{\text{transfer}}\Big\lceil\dfrac{\max\big(0,\ ks\tan(\text{坡度}\perp\theta)-L_{turn}\tan\gamma^g_c\kappa\big)}{\Delta h_{lap}}\Big\rceil T_{lap}$：沿等高線掃描時相鄰條帶目標高差（A 5° 坡、skip-3 ⇒ 41 m，U 型 3° 只能得 ≈32 m ⇒ 每個上坡轉場 1 圈螺旋 +58 s）計入代理；並反映於 §7.6 $s^{turn}$。
- 迴轉帶檢查（D38）：胞在掃描方向兩端外延 $d_{turn,c}=\max\{R_{eff},\sqrt{4R_{eff}^2-(R_{eff}+s_c/2)^2}+R_{eff}\}+e_{ext,c}$（Ω 超出端線量：A@150 m 487 m、A@120 m 501 m、B 288 m，遠大於舊值 $R+e_{ext}$）、側向外延 $R_{eff}-s_c/2$（僅 $s<2R$ 之首末條帶；A 121 m、B 69 m）之迴轉帶落入禁區／keep-out → 該方向 $+\text{pen}_{keepout}$（$=$ 繞行估計時間），若禁區與端線距離 $<R_{eff}+e_{ext}$（連 U 型也放不下）→ 該方向不可行。實際 $\vec t^{trans}$ 於 §8.3 以掃掠足跡逐對檢查。
- 側風方向直線往返因子 $2L/\sqrt{V_a^2-W^2}$ 恆小於順逆風 $2LV_a/(V_a^2-W^2)$，故 $\beta\pm90°$ 常勝出於強風。

### 7.4 條帶產生（sensor-driven、conservative-AGL；D3）

掃描座標：$u$ 沿 $\hat t(\theta)$、$v$ 沿 $\hat n(\theta)$；胞 $v$ 範圍 $[v_{\min},v_{\max}]$，$W_q=v_{\max}-v_{\min}$。

```
LAY_STRIPS(Q, θ, c) -> strips:
  h0 = h_op_c(Q); v_base = v_min(Q); l = 1; strips = []
  while strips == [] or strips[-1].v_hi < v_max(Q):        # 以「前一條上緣」判斷（舊條件 v_base < v_max 會在 W∈(1260,1312] 多鋪 1 條）
      h = h0;  B_prev = None; h_B = h0
      repeat up to 3:                                     # 超集帶單調收縮：任一次終止皆保守
          w = w_c(h);  band = [v_base, v_base + w]
          if B_prev is None: B_prev = band; h_B = h       # 第一次以最寬帶為超集，記錄其高度 h_B
          sub = clip(Q, band); if sub empty: break outer
          u_lo = min_u(sub) − e_ext_c ; u_hi = max_u(sub) + e_ext_c
          centerline = {v = v_base + w/2, u ∈ [u_lo, u_hi]}
          prof = LOCAL_ENVELOPE(centerline ± R_eff 延伸, c, corridor = B_prev)   # §11.3：z_g^max 取「超集帶」B_prev；延伸段只供爬降空間
          h_min = min_{s ∈ [u_lo, u_hi]} (prof.z(s) − z_g^max_{B_prev}(s))       # 只在條帶本身（含 e_ext）取 min，不含胞外延伸段
          if h_min ≥ h − ε_h: break
          h = h_min                                        # 只會變小；band ⊂ B_prev
      w = w_c(min(h_min, h_B))                             # 寬度永不超過超集帶：h_min > h_B 時（包絡整段抬高）多出的寬度未經檢查，不採用
      strips.append(Strip(l, pL=(u_lo, v_base+w/2), pR=(u_hi, v_base+w/2), w, h_min, v_lo=v_base, v_hi=v_base+w, θ_LR=θ))
      v_base += w·(1 − o_side_c)                           # 下一帶下緣 = 本帶上緣 − 重疊 ⇒ 帶序列無縫
      l += 1
  Δ = strips[-1].v_hi − v_max(Q)                           # 最後一條溢出 ≥ 0
  if Δ > 0: 所有 strips 的 v 座標 −= Δ/2                     # 整體置中平移：覆蓋 [v_min−Δ/2, v_max+Δ/2] ⊇ [v_min, v_max]，間距不變
            重算各條 h_min（走廊移動 ≤ Δ/2）；若任一條 h_min 下降 → 自該條起丟棄、以新 v_base 重新進入 while 迴圈
            （重鋪直到 strips[-1].v_hi ≥ v_max；重鋪後不再平移，至多 1 輪）
  assert strips_union_footprint(strips) ⊇ Q                # 後置斷言（覆蓋 §7.7）
  return strips
```

若想利用包絡抬高帶來的額外寬度（$h_{\min}>h_B$），須以新帶重跑一次 LOCAL_ENVELOPE 確認（選項 `widen_on_lift=False`）。

對應論文：Eq.(6)(7) 的 $\Delta w^T_l=(l-1)(w-w_{ovl})$ 在本設計為 $\sum_{j<l}(w_j-o_{side}w_j)$（逐條變寬）；Eq.(8)(9) 的端點 $\min_u/\max_u\{c^{LT},c^{LB},\{v_m:I=1\}\}$ 即 `min_u(sub)`／`max_u(sub)`。等寬時 $n_l=\lceil(W_q-(w-s))/s\rceil$（與迴圈一致：A $W_q=1270$ → 8 條，跨度 1312.9）。走廊地形 $z_g^{\max}(s)=\max_{v\in B}z_g(s,v)$，橫向取樣 $\Delta v=\min(10\text{ m},w/8)$。掃描座標原點 `sweep_origin` = 胞在 $(\hat t,\hat n)$ 投影的 $(u_{\min},v_{\min})$ 角點，存於 `Cell` 並複製到每條 `Strip`；$v$ 沿右法向 $\hat n$（$\theta=90°$ 時 $v=-y$）。

### 7.5 殘帶指派（residual band handoff；D17）

僅 $\sigma_k=\text{any}$。A 鋪 $k_A$ 條後頂緣 $v_{top}=v_{\min}+(k_A-1)s_A+w_A$；殘帶子胞 $Q^{res}=\mathrm{clip}(Q,\{v\ge v_{top}-\delta_{edge}\})$——**只在 $v$ 方向擴 $\delta_{edge}$**（與 A 子胞重疊 $\delta_{edge}$），$u$ 方向沿用胞邊界（胞已外擴 $\delta_{edge}$，不再疊加）；由 B 以自身條帶重鋪。$k_A\in\{0,\dots,n_A\}$ 由配置層決定（§9.5：候選對 $k\in\{0,\lceil n_A/2\rceil,n_A\}\cup$ 估計前 3 名進入 §9.4 拍賣）；$k_A=n_A$ 即 A 獨佔、$k_A=0$ 即 B 獨佔。$\sigma_k\in\{\text{lidar},\text{both}\}$ 不允許（LiDAR 需求不可由相機替代）。混編胞（同胞內兩構型各自條帶）**不允許**。

### 7.6 構型適配度（feed §9）

$$S^c_q=\kappa(\sigma_k,c)\prod_{H\in\{H2..H7\}}\mathbb 1_H\cdot\big[\nu_1s^{width}+\nu_2s^{area}+\nu_3s^{dist}+\nu_4s^{turn}+\nu_5s^{endur}+\nu_6s^{gsd}\big]\in[0,1]$$

$s^{width}=1-\mathrm{frac}(W_q/s_c)s_c/W_q$（整除度）；$s^{area}_A=\mathrm{sat}(A_q/A^*)$，$s^{area}_B=1-s^{area}_A$（$A^*=5\text{ km}\times s_A$）；$s^{dist}_A=\mathrm{sat}(d_q/\rho^{sym}_B)$，$s^{dist}_B=1-s^{dist}_A$；$s^{turn}=1-\big[(n_q-1)t_{turn}+\text{pen}_{\Delta z}\big]/T^c_q$ 再乘 $\big(1-\tfrac12\,\mathrm{sat}(d_{turn,c}/L_q)\big)$（Ω 超出端線量相對條帶長的懲罰，D38）；$s^{endur}=1-T^c_q/T^{use}_c$；$s^{gsd}$：rgb 需求時 B 為 1、A 依機上 RGB 硬體之 GSD 達成率（A 無 RGB 硬體 → $\kappa=0$）；另乘 $\mathbb 1[w_{q,\min}\ge2R_{eff,c}]\cdot0.5+0.5$（胞內無法 U 型迴轉之懲罰）。$\nu=(0.15,0.15,0.20,0.20,0.20,0.10)$。$\mathbb 1_{H4}$ 於此以 $\lambda=\min(n_U^c,n_q)$（最有利）為濾網（§9.3），避免與 $\lambda$ 的循環依賴。

### 7.7 覆蓋完整性論證

1. 分割精確：$\{Q_q\}$ 為 $P$ 的分割（split/merge 不改變並集），$Z\subseteq P$（D31）。
2. 胞內：帶序列自 $v_{\min}$（置中平移後 $\le v_{\min}$）起、相鄰帶重疊 $o_{side}w$、至 $v_{hi}\ge v_{\max}$ 止 ⇒ 帶覆蓋 $[v_{\min},v_{\max}]$；每帶中心線 $u$ 範圍涵蓋 `sub` 極值 ⇒ 足跡矩形 $\supseteq$ sub；實際足跡寬 $w_c(AGL(s))\ge w_c(h_{\min,l})=w$，因 OPER 樣本 $z\ge z^*\ge z_g^{\max}{}_{w/2}+h_{\min,l}$（D36，§11.4 不變量）。
3. 胞間：各胞外擴 $\delta_{edge}$ 互疊 $2\delta_{edge}$（僅區域內部邊界；禁區邊界不外擴，D41），吸收定位／追蹤／DEM 誤差。
4. DEM 保守：AGL 以走廊最高地形計，任一橫向點真實 AGL $\ge h_{\min,l}$。
⇒ $\bigcup\text{footprint}(L_{q,l})\supseteq\bigcup_qQ_q\supseteq Z_k$。驗證由 sim.py 5 m 柵格覆蓋率 ≥99.5% 把關。

---

## 8 三類路徑與時間成本

### 8.1 Dubins 狀態與時間積分

狀態 $S=(x,y,\theta,R_{eff,c})$（航跡方位；autopilot 自行偏流）。任兩狀態取 6 word（LSL, RSR, LSR, RSL, RLR, LRL）最短且通過 keep-out 者為 $D(S_m\to S_n)$；時間

$$t(D)=\sum_{\text{直線}}\frac{L}{V_g^*(\theta)}+\sum_{\text{弧}}\sum_j\frac{R_{eff}\Delta\theta_j}{V_g^*(\theta_j)}\quad(\Delta\theta=5°\text{ 數值積分；D14})$$

例（A、$W=5$）：整圈平均 $1/V_g$ 為 $1/V_a$ 的 1.043 倍；skip-3 U 型（$s=157.5$：701 m）時間 33.9 s，其中半圓 30.5 s（$L/V_a$ 為 29.2 s）；$s=168.0$（@160 m AGL）：732.5 m、35.5 s。

### 8.2 端點狀態與二元／四型成本（論文 Eq.10–12 時間版；D4、D34）

作業路徑 $L_l$：$p^L,p^R$、$\theta_{LR}$（自 $p^L$ 指向 $p^R$）、$\theta_{RL}=\theta_{LR}+\pi$。四個狀態：

| 記號 | 位置 | 方位 | 意義 |
|---|---|---|---|
| $S^L_l$ | $p^L_l$ | $\theta_{LR}$ | 自左端進入、向右掃 |
| $S^{LR}_l$ | $p^R_l$ | $\theta_{LR}$ | 向右掃完在右端 |
| $S^R_l$ | $p^R_l$ | $\theta_{RL}$ | 自右端進入、向左掃 |
| $S^{RL}_l$ | $p^L_l$ | $\theta_{RL}$ | 向左掃完在左端 |

**索引約定（全規格與契約一致）**：掃描方向 $dir\in\{-1,+1\}$（$-1$=LR、$+1$=RL）、索引 $idx=(dir+1)/2$（0 ↔ LR、1 ↔ RL），所有陣列的索引語意皆為「**該條帶自身的掃描方向**」。

$$\vec t^{oper}_{l}[idx]=\frac{|L_l|}{V_g^*(\theta_{idx})},\qquad \vec t^{in}_{i,l}[idx]=\begin{cases}t(D(S_i\to S^L_l))&idx=0\\ t(D(S_i\to S^R_l))&idx=1\end{cases}$$

轉移擴為 **4 型** $\vec t^{trans}_{l,l'}[idx_l,idx_{l'}]$（$k=2\,idx_l+idx_{l'}$）：

| $k$ | $(idx_l,idx_{l'})$ | 起訖狀態 | 型式 |
|---|---|---|---|
| 1 | (0,1) | $S^{LR}_l\to S^R_{l'}$ | 反向（論文 $e=-1$ 分量） |
| 2 | (1,0) | $S^{RL}_l\to S^L_{l'}$ | 反向（論文 $e=+1$ 分量） |
| 0 | (0,0) | $S^{LR}_l\to S^L_{l'}$ | **同向（racetrack）**：向右掃完 $l$，胞外回飛，再自左端向右掃 $l'$ |
| 3 | (1,1) | $S^{RL}_l\to S^R_{l'}$ | 同向 |

論文 Eq.(25)（連續轉移方向相反）放寬為「**作業方向 = 轉移終端狀態之方向**」（Eq.(26) 原樣保留，Eq.(23)/(24) 亦保留）；反向轉場即論文原型，同向轉場使單向可行的胞（§4.4：順風 $V_g>V_g^{cap}$、只能逆風掃）仍可被覆蓋——舊版反向鎖定下，B 於 GSD 2 cm、風 8 m/s 時掃完 1 條就 `cand=∅` FAIL。矩陣 $\mathbf T_i$（$(n+1)\times n\times2$ 進入列、$n\times n\times4$ 轉移）與 $\mathbf E_i$ 結構同論文 Eq.(13)(14)；不可行元素 $=\infty$。$S_i$ = 起飛走廊出口狀態 $S_{acc}$（加速段末，§3.4／§11.6）或爬升盤旋出口 $S_{exit}(\vartheta^*)$。`legacy_mode` 只保留 $k\in\{1,2\}$。

### 8.3 轉場幾何：$s<2R$（分析與單元測試；D5）

| 圖樣 | 條件 | 長度 | 超出端線 | 側向溢出 |
|---|---|---|---|---|
| U 型（LSL/RSR） | $ks\ge2R$ | $\pi R+(ks-2R)$ | $R$ | 0 |
| Ω／燈泡（RLR/LRL） | $s<2R$ | $R\big(\pi+4\theta_\Omega\big)$，$\theta_\Omega=\arccos\frac{s+2R}{4R}$ | $\sqrt{4R^2-(R+s/2)^2}+R$ | $R-s/2$（兩側） |
| 環型（LSL 於 $s<2R$） | $s<2R$ | $3\pi R+(2R-s)$ | $R$ | $2R-s$ |
| 同向 racetrack（LSL/RSR，D34） | 任意 $s$ | $\approx2\pi R+|L_l|+\ldots$（由求解器） | $R$（兩端） | 0 |

$s=0$：Ω $7.33R$、環 $11.42R$；$s=2R$：三者皆 $\pi R$。Skip-$k$：$k=\lceil2R_{eff}/s\rceil$。數值（$W=0$）：A（$R=200$，$s=126$ @120 m AGL）$k=4$、Ω 1.31 km、U 732 m；A（$s=157.5$ @150 m）$k=3$、Ω 1268 m、U 701 m；A（$s=168.0$ @160 m）$k=3$、Ω 1253 m、U 732 m；B（$R=120$，$s=103$–105）$k=3$、Ω 749 m、U 445–452 m。Ω 超出端線量（D38）：A@150 486.9 m、A@120 501 m、B 288 m；側向溢出 A 121 m、B 69 m——皆遠大於 $R$，故迴轉帶檢查與 H2 一律用 $d_{turn}$。**實作不硬編碼圖樣**：$\vec t^{trans}_{l,l'}$ 對所有 $(l,l')$、4 型由 6-word 求解器求得，IDP 依時間自動選出 skip-$k$ 序列或局部 Ω；閉式只用於 §7.3 代理、複雜度上界與測試（`test_transfer_closed_forms`）。轉場掃掠足跡（路徑 ⊕ $b_{trk}$）須通過 keep-out／禁區檢查，否則 $\infty$。

### 8.4 高度差驅動的轉移延長與轉移能量

相鄰作業路徑目標高度差 $\Delta z$（地形或分層）需水平距離 $\ge|\Delta z|/\tan\gamma^g$：

$$t^{trans}\leftarrow\max\Big(t(D),\ \frac{|\Delta z|}{V_a\sin\gamma}\Big),\qquad n_{lap}=\Big\lceil\frac{|\Delta z|-|D|\tan\gamma^g}{\Delta h_{lap}}\Big\rceil^+\ (\text{螺旋圈數，}\Delta h_{lap}\text{ 見 §11.6})$$

螺旋圓心置於迴轉帶內、通過 keep-out 檢查。

**轉移能量**（不再以 $P_ct$ 計；A 爬升功率 $P_\uparrow=511+453=964$ W，+89%，$\Delta z=50$ m 每次少算 6.7 Wh、山區 20 次轉場 ≈135 Wh = $E^{use}_A$ 的 17%）：

$$e^{trans}_{l,m}=k_{unc}\Big[P_c\,t(D)+(P_\uparrow-P_c)\frac{\max(0,\Delta z)}{V_a\sin\gamma_c}-(P_c-P_\downarrow)\frac{\max(0,-\Delta z)}{V_a\sin\gamma_d}+P_\uparrow n_{lap}T_{lap}\Big]$$

`entry_e` 同式（$\Delta z=z_{entry}-z_{S_i}$，含 §11.6 盤旋圈數）；`rtb_e` 同式（下降側）。

### 8.5 Dubins 離散化（autopilot）與 keep-out（D27）

外切多邊形：$\Delta\psi=\min\big(2\arccos\frac{R_{eff}}{R_{eff}+e_{\max}},\ 30°\big)$（$e_{\max}=3$ m → A 19.8°、B 25.6°）；切點 $P_{c,j}$（論文 Eq.28）、航點 $P_{a,j}$ 為相鄰切線交點（Eq.29）。支援弧命令時（ArduPilot `NAV_LOITER_TO_ALT`／PX4 orbit）輸出圓心／半徑／方向。每航點附 $z$、$V_a^*$、`sensor_on`、接受半徑 $r_{acc}=\min(R_{eff}/2,30)$ m。

keep-out：$\mathcal K_o=(\text{geom}_o\oplus\mathrm{disc}(b_h),\ z_{top,o}+b_v)$，點→圓、線→膠囊、面→圓角偏移；$b_h=50$ m（含 $e_{xt}$）、$b_v=30$ m。作業路徑不繞行（覆蓋要求），衝突以抬高解決（§11.2 之 $z_{lo}$ 含障礙頂 $+b_v$）；轉移／進入／返航做水平繞行（2.5D：只有當該處 $z<z_{top}+b_v$ 才進圖）。

### 8.6 障礙繞行（D30）

圓形（論文 Eq.27 推廣）：$r_D=\max(R_T+b_h,R_{eff})$，安全點為自起點至圓 $r_D$ 的切點，取離原路徑較近側，$Y_1=D(S_i\to S_{D,1})+D(S_{D,1}\to S_{End})$。多邊形：以 $\rho=\max(b_h,R_{eff})$ 做圓角偏移（Minkowski ⊕ disc），邊界僅含直線與半徑 $\rho\ge R_{eff}$ 之圓弧 ⇒ 沿邊界飛行本身滿足曲率限制且淨空 $\ge b_h$。

```
AVOID(S_a, S_b, K, z_hint) -> DubinsPath:
  D0 = dubins6(S_a, S_b, R_eff); if clear(D0 ⊕ b_trk, K_active(z_hint)): return D0
  G  = tangent_graph({circle(S_a), circle(S_b)} ∪ rounded_offset(K_active, ρ))    # 節點=切點，邊=公切線/弧
  path = dijkstra(G, weight = time_with_wind)                                     # 邊權以 §8.1 積分
  if path is None: 備援：visibility_graph(K ⊕ (b_h + R_eff(sec(Δ_max/2)−1))) + Dubins 接合（頂點緩衝補償弧切角）
  return concat(path)                                                             # 曲率 ≤ 1/R_eff
```

### 8.7 返航路徑（第四類）

$$t^{RTB}_i(S;a)=t\big(\text{AVOID}(S\to S^{app}_a)\big)+t_\downarrow^{extra}+t_{acc}+t^{in}_{tr}+t_{dn}+t_{hov,res}$$
$S^{app}_a$ = 降落走廊入口（§11.7：下風側、FW_DECEL 起點 $P_{tc}-d_{acc}\hat t(\theta_{app})$、方位 $\beta$、$H_{tc}$）；下降距離不足時加下降盤旋。$e^{RTB}$ 以 §8.4 公式（含下降功率、$k_{unc}$、實際風三角）。

**落點政策（D40）**：$a=H$（或任務指定之回收點 `recovery_site`）優先；只有 $H$ 不可行（能量／站點風／走廊淨空／預約）才依 $t^{RTB}$ 遞增試備降點，並在 `UAVPlan.warnings` 標 `planned_landing_at_alternate`；makespan 的 $C_i$ 與 H4 的 $e^{RTB}$ 一律以 $H$ 計。舊版 $\arg\min_{a\in\mathcal A\cup\{H\}}$ 會把備降點當常規落點（makespan 被人為壓低、地面回收未建模、與多架次換電只在 $H$ 矛盾）。

### 8.8 可飛性論證

(i) 曲線段半徑 $\ge R_{eff}\ge R_{theo}(1+W/V_a)^2$ ⇒ 任意相位傾角 $\le\phi_{\max}$；(ii) 直線段 $V_g$ 存在（$|W_\perp|<V_a$）；(iii) 高度坡度以地面坡度 $\gamma^g$ 限制 ⇒ 空氣坡度 $\le3°/4°$；(iv) 弧段爬升以 $\gamma\cos\phi_{\max}$（A 2.82°、B 2.72°）；(v) 離散偏差 $\le e_{\max}$；(vi) $V_a^*\in[V_{a,\min}(m),V_{a,c}]$；(vii) 陣風：$R_{eff}(W+G)$ 作為 keep-out 掃掠足跡的額外半寬檢查（不改變規劃半徑）。

### 8.9 成本快取（CostCache）

```
CostCache:                                  # 每構型獨立索引：strip_ids[c], idx[c]（any 胞的 A、B 條帶不混在同一維度）
  op[c]    : (n_c, 2)       s   [l, idx]：l 以 idx 方向掃的作業時間；只依構型（V_a*, 風）  geometry-free
  tr[c]    : (n_c, n_c, 4)  s   [l, m, k]，k = 2·idx_l + idx_m（§8.2 四型；幾何層 Dubins 長度／word 與時間層分離）
  tr_e[c]  : (n_c, n_c, 4)  Wh  §8.4 含爬升功率
  entry[i] : (n_c, 2)       s   [l, idx]：自 S_i 進入後以 idx 方向掃 l（idx=0 → 進 p^L）；含暫定起飛走廊之 n_lap·T_lap
  entry_e[i]: (n_c, 2)      Wh
  rtb[c,a] : (n_c, 2)       s   [l, idx_l]：l 以 idx_l 掃完（idx 0 → 在 p^R 朝 θ_LR；1 → 在 p^L 朝 θ_RL）至 a 之 S^app
  energy   : EnergyBudgetTable  (c, strip, idx, site) → Wh（AVOID+ENVELOPE 真實值，§5.5）
  W_R[c]   : float          §3.2 所用風速；wind_key : int   風更新只重算時間層；W_R 使 R_eff 變化 >5% 才重算幾何層
  static_ok[c]: (n_c, 2) bool   H4 靜態濾網（§10.3）：r·E_nom + e_alt(端點) + e(H→端點 直線最壞) ≤ η_T·E_avail,min − e_TO
```
$\mathbf T_i$ 由切片即時組合：`T_row0[i][l, idx] = entry[i][l, idx] + op[c][l, idx]`、`T_transfer[c][l, m, k] = tr[c][l, m, k] + op[c][m, idx_m]`（$idx_m=k\bmod2$），不實體化 $n_U$ 份。增量方法 `add_strips(c, new)`／`remove_strips(c, ids)`（殘帶重鋪、新區域）只補算受影響列／行。**entry 與盤旋**：`build` 前先以「暫定入口 = 各條帶端點中最近者」跑一次 TAKEOFF_CORRIDOR 得 $n_{lap}$ 與 $S_{exit}$ 估計，`entry[i]` 含 $n_{lap}T_{lap}$、`entry_e` 含其能量；IDP 後首條帶若改變則 `refresh_entry` 一次（§11.6）。

---

## 9 任務配置（Allocation）

成本一律為時間；目標 min-max 落地時刻（含返航）。輸入：所有胞（含殘帶子胞、`both` 之雙套條帶）、各機初始狀態與能量。

### 9.1 硬限制

- H1 感測：$\kappa(\sigma_k,c)=1$（以機上 `available_kinds`）。
- H2 鏈路／半徑：$(Q_q\oplus\mathrm{disc}(d_{turn,c}))\subset\mathcal P_{link,c}\cap\mathrm{Disc}(H,\rho^{sym}_c)$（下緣；迴轉帶含 Ω 超出量，D38；§6.2 步 7 已先內縮 $d_{turn}$ 再裁切，驗證失敗時自動改用迴轉帶朝內的方向候選）。
- H3 時間窗：胞到達 $\ge t^{open}_k$、完成 $\le t^{close}_k$；所有起降在空域全域窗內（D22）。**提早到達**以「延後起飛 $\Delta=t^{open}-t_{arr}$」處理（零能量，改 $t^0_i$，重排走廊 §9.7）；不可延後（同機前有其他胞）時插入等待盤旋並計能量；IDP 後對每胞驗證完成時刻 $\le t^{close}$，超出者自尾端移除該胞條帶為 leftover。
- H4 能量：單機份額 $e^{TO}+e^{in}+e^{oper}/\lambda+e^{trans}+e^{RTB}(\to H)+e^{LDG}\le\eta_TE^{avail}-rE_{nom}$ 且 §5.5 逐點；於 $S^c_q$ 中以 $\lambda=\min(n_U^c,n_q)$ 作濾網。
- H5 感測速度：$\exists\theta$ 使 $V_a^*$ 存在（§4.4；單向可行即可，D34）。
- H6 高度帶：$\Delta h_{band}\ge20$ m（§4.6，胞層級）。
- H7 幾何：$w_{q,\min}\ge s_c$（否則單條帶）；迴轉帶（D38）不落禁區（否則需繞行成本）。

### 9.2 配置指標（擴充論文 Eq.30；時間版）

$$J^c_q=\omega_1\frac{\sum_{i\in U^c}\min_lt^{in}_{i,q,l}}{\sum_{q'}\sum_i\min_lt^{in}_{i,q',l}}+\omega_2\frac{n^c_q}{\sum_{q'}n^c_{q'}}+\omega_3\frac{T^{oper,c}_q}{\sum_{q'}T^{oper,c}_{q'}}+\omega_4\frac{val_q}{\sum val}+\omega_5\frac{T^{RTB,c}_q}{\sum T^{RTB,c}}+\omega_6\frac{S^c_q}{\sum S^c}$$

$T^{oper,c}_q=\sum_l\max(t^{LR}_l+t^{RL}_{l+1},\,t^{RL}_l+t^{LR}_{l+1})$（論文 Eq.30 註解之時間版；逐對取 max 相加，每條帶計入兩次，**只作 $J$ 的比值用**）。能量下限、$\tilde T$ 與 makespan 估計一律用 `Cell.T_single_s` $=$ `exact_time`（兩向平均＋轉場，§7.1），不得引用 $T^{oper}$。$\omega=(0.2,0.15,0.2,0.15,0.1,0.2)$；$\omega_5=\omega_6=0$、其餘 0.25 即論文。

### 9.3 構型指派與架數（每構型獨立 Eq.31；D16）

```
ASSIGN_CONFIG_AND_COUNT(cells, U_A, U_B):
  for q: feasible = {c : S^c_q > 0}      # S 內 H4 以 λ=min(n_U^c, n_q) 濾網（無循環依賴）
        if ∅: mark UNCOVERABLE
        score_c = S^c_q / T̃^c_q,  T̃ = (t_in + T_single + t_RTB)/T_use^c ; c*_q = argmax; c2_q = 次選
  for c in {A,B}:
     Q_c = {q : c*_q = c}
     λ^c_q = clip( round(n_U^c · J^c_q / Σ_{Q_c} J), 0, min(n_U^c, n^c_q) )
     denom = κ_load·T_use^c − t_in − t_RTB                                    # κ_load = 0.85
     if denom ≤ 0: (q,c) infeasible(reason="energy_overhead")（例：B、胞距 30 km、W=5：2978 s < 2143+1250）；改派 c2_q 或 UNCOVERABLE
     λ_floor = ceil(T_single^c_q / denom)                                     # 能量下限
     if λ_floor > min(n_U^c, n^c_q):                                          # 架數不足：部分覆蓋而非整胞消失
         若 multi_sortie: 交 §9.6；否則以能量預算依條帶順序取前 k 條（Σ 前 k 條之 (t_oper+t_trans) ≤ λ_avail·denom），
         其餘條帶記 UNCOVERABLE(reason="energy", strip_ids)；λ = min(n_U^c, n_q)
     else: λ^c_q = max(λ^c_q, λ_floor)
     if Σλ > n_U^c: 依 J 由小到大遞減 λ（不低於能量下限）；仍超出 → 多架次（§9.6）或 J 最小之胞改派 c2_q
  return {c*_q, λ^c_q}          # λ=0 的胞由拍賣階段以「一機多胞」承接
```

### 9.4 貪婪 → 拍賣 → 局部改善

```
ALLOCATE(cells, fleet, cache, budget) -> teams:
  Phase 1 貪婪（論文 Algorithm 1 時間版）：
     cells 依 J^{c*}_q 降序；對每胞選 λ 架「進入時間最短且 H1–H7 可行」之未指派同構型機；不足則 λ ← |cand|
  Phase 2 拍賣（一機可承擔多胞；小胞串接；殘帶候選對）：
     open = 未指派或 λ=0 之胞 ∪ {any 胞之候選對 (A 子胞(k), B 殘帶(k)) : k ∈ {0, ⌈n_A/2⌉, n_A} ∪ top3_k Ĉ}
     while open ≠ ∅:
        for q in open, i in fleet with κ(σ_q,c(i))=1, pos in insertion positions of plan[i]:
            if not hard_feasible(q, i) or not ENERGY_FEASIBLE(plan[i] with q@pos) or not TIME_WINDOW_OK(q, arrival): continue
            Δ = MAKESPAN_AFTER_INSERT(plan, i, q, pos) − C_max(plan)      # 候選對：對 (A 子胞, B 殘帶) 同時評估（兩機各一插入位置），取兩者之 max
            bid = (Δ, C_i_after, −S^{c(i)}_q)                       # 字典序
        best = min bid
        if best is None: 所有仍 open 之胞記 UNCOVERABLE(reason="no_feasible_insertion")（多架次時先交 §9.6）; break
        apply best; open.remove(q)（候選對：同時移除同胞的其他 k 候選）
  Phase 3 局部改善（min-max）：
     repeat ≤ N_ls: i_max = argmax C_i；對 q ∈ plan[i_max]、j ≠ i_max 嘗試 MOVE(q→j)、SWAP(q↔q'∈plan[j])；
            接受條件：hard_feasible(H1–H7，含感測相容) 且 λ ≥ 能量下限 且 C_max 降（tie：ΣC_i 降）
     any 胞局部微調 k_A：對每個 any 胞以 MOVE 條帶邊界 ±1 嘗試（一次 pass）
  Phase 4 BALANCE：估 T̂_i = t^0_i + t_in + T_work/λ + t_RTB；自 argmax 團隊移一架至可行（hard_feasible、同 cls、移出後 λ ≥ 能量下限）
                   且使 max T̂ 降幅最大之團隊，≤ n_U 次
  Phase 5 團隊合併：同構型、胞相鄰、單機團隊 → 合併為多胞團隊（事件驅動 IDP 於聯集條帶上執行，各機條帶數可不等，D33）
  return teams  # Team(id, uav_ids, cell_ids, strip_ids, type c, t0_i)
```
`MAKESPAN_AFTER_INSERT`：進入 Dubins 時間（`entry[l, idx]` 兩側擇優）+ $T^c_q$（單機，`T_single_s`）+ 至下一胞 Dubins + 更新 RTB：**RTB 必須取 `rtb[c,H][last, idx_last]` 之正確終端側**（終端側由條帶數奇偶與進入側決定，不可固定右端）。IDP 結果與估計偏差 >10% → 回饋一次 Phase 3。

### 9.5 殘帶 $k_A$（`any` 胞、A 與 B 同時可用；併入拍賣）

$k_A$ 不再逐胞獨立枚舉（多個 `any` 胞各自把同一架 A 當成全職會使每胞都選「A 多做」；$\lambda_A>1$ 時亦未除以 $\lambda$），而是以候選對進入 §9.4 Phase 2：$\hat C_A(k)$、$\hat C_B(n_B(k))$ 以 `T_single_s`＋進入／RTB（正確終端側）估計，先取 $k\in\{0,\lceil n_A/2\rceil,n_A\}\cup$ 前 3 名，投標時以 `MAKESPAN_AFTER_INSERT` 對 (A 子胞, B 殘帶) 同時評估、選 $\Delta$ 最小者；Phase 3 再局部微調 $k_A$。例（3 km × 1.2 km、$W=5$ m/s、1A+1B、$h_{op,A}=160$、$h_{op,B}=100$）：$k_A=5,n_B=3$ → makespan 28.3 min，A 獨佔 34.7、B 獨佔 51.0 min；$k_A=6$（$n_B=2$）28.4 min 僅差 6 s，IDP 精算後兩者相差 <1 s（契約 §W.4）。

### 9.6 多架次（`multi_sortie=True`）

同機換電（$t_{swap}=600$ s）後再出：加入虛擬機 $U_i^{(2)}$（起點基地、可用時刻 $C_i^{(1)}+t_{swap}$、能量滿電或指定）；**第一架次 `landing_site == home` 為硬條件**（D40）。大胞以 `SPLIT_FOR_SORTIES` 依條帶順序切成能量可行塊（budget $=\kappa_{load}T_{use}-t_{in}-t_{RTB}$）。僅在「所有機皆滿載仍有未覆蓋」時啟用；第二架次完工時間計入 $C_{\max}$。

### 9.7 起飛排程

同站起飛間隔 $\Delta t_{dep}=60$ s；順序 = 估計完工時刻 $\hat C_i$ 降序（最長者先起，直接減小 makespan；tie → 進入時間長者先）；H3 提早到達之延後起飛於此加入；$t^0_i=k_i\Delta t_{dep}+\Delta_i$ 饋入 IDP 初始狀態；降落預約見 §13.4（衝突時零能量的起飛換序／延遲優先於盤旋）。

---

## 10 異構 IDP（H-IDP）

### 10.1 狀態與初始化（論文 Eq.32 擴充）

團隊 $\mathcal U_g$（同構型）、條帶集合 $\mathcal L_g$（可跨多胞）。$\Sigma(1)=\{S_i=S_{acc,i}\ (\text{或盤旋出口 }S_{exit}(\vartheta^*)),\ T_i=t^0_i+t^{TO}_i+n_{lap,i}T_{lap},\ E^{rem}_i=\eta_TE^{avail}-e^{TO}-e^{orbit}(n^{worst}_{lap,i}),\ R_i=\emptyset,\ \text{active}_i=\text{True}\}$，$\mathcal L^{NAD}(1)=\mathcal L_g$。$n^{worst}_{lap,i}$ 以最遠條帶端點之進入路徑估算（保守初值，§11.6）；精算後若更差 → 重跑 ENERGY_SAFE_CHECK 並依 §10.5 修復。進入段本身由 `entry[i]`（第 0 列）計入，不在走廊段重複（§11.6）。跨胞轉移由 AVOID 計算（含 §8.4 高度差延長與能量）。

### 10.2 階段成本（取代論文 Eq.33；D28、D33）

**事件驅動分支**：每階段只擴展一架機 $i^*=\arg\min_iT_i$（active 者；tie → `uav_id`），為其選下一條帶與轉場型 $(m,k)$：

$$W(\Sigma,(i^*,m,k))=\max_j\Big[T_j+\mathbb 1[j=i^*]\big(t^{trans}_{l_{i^*}\to m,k}+t^{oper}_{m,idx_m}\big)+\hat t^{RTB}_j\Big]+\epsilon\sum_j\big[\cdots\big]+\mu\frac{t^{trans}+t^{oper}}{T_{i^*}+\tau_0}$$

$\hat t^{RTB}$：直線逆風下界（A* 式可採納啟發）；$\epsilon=0.05$、$\mu=0$（`metric="paper"` 時 $\mu=1$、前兩項 0）、$\tau_0=60$ s。第一階段用 `entry[i*][m, idx]`（兩側擇優：順風出、逆風回時偏好遠端進入）。方向：作業方向 = 轉移終端狀態方向（D34；反向與同向轉場皆為候選，$k\in\{2\,idx_{l},2\,idx_l+1\}$）；不同機不由同條路徑進入（Eq.23）。**同胞多機剪枝**（D18，取代 $|l_i-l_j|s<d_{sep}$——$s\ge103$ m 時恆假）：兩機的迴轉帶（端線外 $d_{turn}$、側向 $ks+R_{eff}$，D38）在時間區間 $[T_i,T_i+t^{trans}+t^{oper}]$ 上重疊 → 剪；U 型轉場橫掃的 $k-1$ 條中間條帶端區與 Ω 側向溢出因此被涵蓋。

### 10.3 能量剪枝（逐機，非逐排列）

候選 $(m,k)$ 剪枝若 $E^{rem}_{i^*}-e^{trans}-e^{oper}<rE_{nom}+\text{budget.min\_rtb}(c,m,idx_m)$ 或 `static_ok[m, idx_m]=False`（§8.9 靜態濾網：$rE_{nom}+e^{alt}_{AVOID}(\text{端點})+e(H\to\text{端點 直線最壞})\le\eta_TE^{avail}_{\min}-e^{TO}$，使無法安全抵達的條帶根本不進候選）。$\text{near}_{i^*}=\emptyset$ → 該機於本階段 $\pi_{i^*}=$RTB：$T_{i^*}\mathrel{+}=t^{RTB}_{i^*}(S_{i^*})$、$E$ 固定、`active=False`（序列保留），**其餘機繼續**；只有所有機皆 inactive 且 $\mathcal L^{NAD}\ne\emptyset$ 才回傳 leftover。（舊版以排列為單位、一機無解即整隊 FAIL，會把其他仍有 40% 能量的機的部分序列一併丟棄。）

### 10.4 Beam-IDP（論文 Algorithm 2 之事件驅動擴充）

```
BEAM_IDP(team, L_g, cache, budget, K=6, B=3, warm_start=None) -> per-UAV sequences:
  beam = [Node(Σ(1), NAD=L_g, R=∅)] (+ warm_start 節點若給定)
  while any(node.NAD ≠ ∅ and any(node.active) for node in beam):
    cand = []
    for node in beam:
      i* = argmin_{i active} T_i
      # k-nearest：依當前狀態 S_{i*}（唯一：條帶 l、掃向 idx_l）對 NAD 中每條 m 取 4 型中相容之 2 型 T_transfer[l,m,k]，
      #            同胞未覆蓋條帶優先進入 near（胞完成偏好），僅當同胞剩餘 < K 時才補鄰胞條帶（跨胞由 AVOID 計）
      near = K 個 (m, k) 最小者；first stage 用 entry[i*][m, idx]（兩側）
      near = [(m,k) for (m,k) in near if not violates Eq.(23) / 迴轉帶時間重疊 / ENERGY_PRUNE(node, i*, m, k)]
      if near = ∅: cand.append(child(node, π_{i*}=RTB)); continue          # 逐機 RTB（§10.3），其餘機下階段繼續
      for (m,k) in near: cand.append(child(node, (i*, m, k), cost = W(node.Σ, (i*,m,k))))
    if cand = ∅: break
    beam = top-B of cand by W（W 已含 max 項、t̂^RTB 與 ε 總和；不再以不含 t̂^RTB 的 max T_i 排序），
           去重鍵 (frozenset(NAD), tuple((strip_i, dir_i) for i))
    if elapsed > realtime_budget_s: B = 1                                          # 超時降級
  best = argmin_{beam} max_i [T_i + t^RTB_i(S_i)]                                  # 真實 Dubins 返航（§8.7 至 H）
  leftover = best.NAD
  for i in team: APPEND_RTB(plan_i)；ENERGY_SAFE_CHECK(route_i) → 失敗依 §10.5 修復
  return {R_i, E_i (Eq.14), T_i, E^rem_i}, leftover
```

階段數 $=n$（每階段一條帶），每階段 $\le B\cdot K$ 子節點（$K$ 已含 2 種轉場型），複雜度 $O(nBK)$；各機條帶數自然不等（跨胞聯集、殘帶、凹胞短條），錯開起飛與重規劃時各機進度不同亦自然處理。反例（舊鎖步）：2 架 A、8 條長（140 s）+16 條短（25 s），鎖步強制 12/12 條 → makespan 1220 s，事件驅動 → 770 s（−37%）；重規劃中 A1 已在胞內、A2 自 10 km 外接手，鎖步使 A2 結束於 880 s，事件驅動由 A1 單獨做完 582 s。`legacy_mode` 保留論文鎖步（每階段每機各取一條、$\Pi=\prod\text{near}_i$，$O(\frac n\lambda BK^\lambda)$）。`APPLY` 更新 $S_{i^*}$（新端點、依 $k$ 之方位）、$T_{i^*}$、$E^{rem}_{i^*}$、決策序列（論文 Eq.22 結構）。**向量化**：`IDPNode` 以 (λ,) 陣列存狀態；`stage_cost`／`energy_prune` 對候選矩陣 `(M, )` 一次計算（numpy fancy-index 讀 `tr/op/budget`），無逐候選 Python 呼叫。

### 10.5 返航附加與能量修復

```
APPEND_RTB(plan_i):                                          # D40：先 H，再備降
  for a in [H (or recovery_site)] + sorted(ALT, key = rtb[c,a][last, idx_last]):
      path = AVOID(S_last → S^app_a) (+ 下降盤旋)；if ENERGY_SAFE_CHECK(route_i + path) ok:
          加入；若 a ≠ H: warnings += planned_landing_at_alternate；return
  return INFEASIBLE(seg, x_j)                                 # 連最近備降點都不可達

REPAIR_ENERGY(team, result):
  # ENERGY_SAFE_CHECK 於樣本 x_j 失敗：E_j 只取決於 x_j 之前的消耗，移除 x_j 之後的條帶對 E_j 與 e^alt(x_j) 無影響，
  # 故不逐條彈出最後一條（舊版會一路彈到含 x_j 的那條、把其後所有可行條帶一併丟掉）
  item_j = 含 x_j 之 SeqItem；截斷 plan_i 至 item_j 之前；tail = item_j 及其後所有條帶
  重跑 BEAM_IDP(team, NAD = tail, warm_start = 各機截斷後序列)   # 其他機可接手 tail；i 自截斷點直接 RTB
  仍有 leftover → ALLOCATE(leftover, A = 受影響集合 §12.1，其他團隊只允許尾端追加) → 多架次或 uncovered 報告
```

### 10.6 跨胞串接與退化

機 $i$ 依配置順序 $q_1\to q_2\to\dots$：前胞末端狀態為後胞初始狀態（第 0 列改為跨胞進入時間，含 §8.4）；候選集含胞完成偏好（§10.4），避免機在胞邊緣時鄰胞條帶「更近」而提前離胞、留下零星條帶。多機共享同一胞時各機初始狀態可不同（論文原生支援）。退化：`legacy_mode`（鎖步、只含反向轉場）、$B=1$、$K=\infty$、`metric="paper"`、單胞、同構、$E=\infty$、$t^{RTB}=0$ → 逐行等於論文 Algorithm 2（時間 = 距離/V）。

---

## 11 三維高度與起降走廊

### 11.1 取樣與每樣本量

沿全架次 2D 航跡以 $d_d=25$ m 取樣，弧段取樣間距 $\min(d_d,R\Delta\psi)$（$R\Delta\psi$ A 69 m、B 54 m $>d_d$，故弧段仍以 25 m 取樣；每段長度 $ds_j$ 為陣列，ENVELOPE 逐段使用）。段型別 ∈ {TO, TRANS_OUT, ACCEL, **CLIMB**, ORBIT, ENTRY, OPER, TRANS, RTB, **DESCENT**, DECEL, TRANS_IN, LAND}（D35）：
- **CLIMB**：自 $P_{tc}$（起飛側釘住點）起，至首個樣本 $j_c$ 使「自 $P_{tc}$ 以 $\tan_c$ 全速爬升之上界線 $u$」$\ge z_g^{\max}(j)+h_{\min fw,c}$ 為止（平地 $(h_{\min fw}-H_{tc,cmd})/\tan\gamma^g_c$：A 未驗證 $(120-100)/0.04246\approx471$ m、B 241 m；驗證後取下緣時 A 942 m／B 724 m，順風 5 m/s；無風 A 764／B 573 m）。
- **DESCENT**：對稱，自最後一個樣本 $j_d$ 使「自降落側 $P_{tc}$ 以 $\tan_d$ 反推之上界線」$\ge z_g^{\max}(j)+h_{\min fw,c}$ 起，至 $P_{tc}$ 止。
- 表 1 的 $H_{tc}<h_{\min fw}$ 使這兩段**必然**以固定翼飛在 $h_{\min fw}$ 以下；舊版對所有段套 $h_{\min fw}$ 使 $a[0]=z_H+120>u[0]=z_H+100$，平地、無風的最簡情境每架次都 INFEASIBLE。

每樣本 $j$：
- 走廊地形 $z_g^{\max}(j)=\max_{|v|\le b_j}z_g$，$b_j=w_l/2$（OPER，該條帶寬）或 $b_{trk}=50$ m（其他）；$z_g^{\min}(j)$ 同理；$z_g^0(j)$ = 正下方。
- 障礙 $z_{obs}(j)=\max\{z_{top,o}+b_v:(x_j,y_j)\in\mathcal K_o\}$。
- 下界：
  - CLIMB／DESCENT：$z_{lo}(j)=\max\{z_g^{\max}+h_{clr},\ z_{obs}\}$（**不含 $h_{\min fw}$**）。
  - ENTRY／TRANS／RTB／ORBIT：$z_{lo}(j)=\max\{z_g^{\max}+h_{clr},\ z_g^{\max}+h_{\min fw,c},\ z_{obs}\}$。
  - OPER（D36）：$z_{lo}(j)=\max\{z_g^{\max}+h_{clr},\ z_g^{\max}+h_{\min fw,c},\ z_{obs},\ z_g^{\max}{}_{w_l/2}(j)+h_{\min,l}\}$——把條帶的保守 AGL 直接寫入下界，使 $z^*$ 本身保證寬度。
  - 走廊段（TO…ACCEL、DECEL…LAND）依 §11.6/11.7 之高度線。
- 上界 $z_{hi}(j)=\min\{z_g^{\min}+h_{\max,c}k_{GSD},\ z_g^0+h_{ceil,c},\ z_{msl,\max}\}$（$h_{\max,c}$ 僅 OPER 段）。
- 目標 $z_{des}(j)=z_g^{\max}+h_{op,c}$（OPER）；TRANS：前後 OPER 目標之線性內插；ENTRY/RTB：$z_g^{\max}+h_{transit,c}$；CLIMB/DESCENT：$z_{des}=+\infty$／同 RTB（即以 $\gamma_c$ 全速爬升、貼 $u$）。
- 坡度限：$\tan_c(j)=\tan\gamma^g_c(\theta_j;m,\kappa_j)$、$\tan_d(j)=\tan\gamma^g_d(\theta_j)$（§3.2；弧段 $\kappa=\cos\phi_{\max}$）。

### 11.2 坡度受限包絡（ENVELOPE；D2）

$$z^*(j)=\max\Big\{\underbrace{\max_{j'\le j}\big[z_{lo}(j')-\textstyle\sum_{m=j'}^{j-1}ds_m\tan_d(m)\big]}_{a:\ \text{波峰後只能以 }\gamma_d\text{ 下降}},\ \underbrace{\max_{j'\ge j}\big[z_{lo}(j')-\textstyle\sum_{m=j}^{j'-1}ds_m\tan_c(m)\big]}_{b:\ \text{波峰前必須以 }\gamma_c\text{ 提前爬升}}\Big\}$$

```
ENVELOPE(z_lo[0..N], z_hi[0..N], tan_c[0..N], tan_d[0..N], ds[0..N-1], pin_start=None, pin_end=None):
  # ds[j] = 樣本 j 到 j+1 的弧長（直線段 d_d、弧段 min(d_d, RΔψ)）；純量 d 只是 ds 全等的特例
  a[0] = max(z_lo[0], pin_start or -inf)
  for j in 1..N:    a[j] = max(z_lo[j], a[j-1] - ds[j-1]*tan_d[j-1])     # 前向：下降角限制
  b[N] = max(z_lo[N], pin_end or -inf)
  for j in N-1..0:  b[j] = max(z_lo[j], b[j+1] - ds[j]*tan_c[j])         # 後向：爬升角限制
  z* = max(a, b)
  # 上界：自釘住起點不能爬得更高；自釘住終點反推不能過高（落地前必須降得下來）
  u = z_hi.copy()
  if pin_start: u[0] = min(u[0], pin_start); for j in 1..N:   u[j] = min(u[j], u[j-1] + ds[j-1]*tan_c[j-1])
  if pin_end:   u[N] = min(u[N], pin_end);   for j in N-1..0: u[j] = min(u[j], u[j+1] + ds[j]*tan_d[j])
  if any(z*[j] > u[j]): return INFEASIBLE(first j, z*[j]-u[j])
  return z*, u
```

可行性：$a$ 只在下降側受限、$b$ 只在爬升側受限；$\max(a,b)$ 在任一點的左斜率 $\ge-\tan_d$（來自 $a$ 的遞迴或 $b$ 上升）且右斜率 $\le\tan_c$，故 $z^*$ 是滿足 $z\ge z_{lo}$、$-\tan_d\le dz/ds\le\tan_c$ 的**逐點下確界且本身可行**。論文 Eq.36 為其單向貪婪近似（非等價）。

### 11.3 局部包絡（供 §7.4）

`LOCAL_ENVELOPE(centerline ± R_eff, c, corridor)`：同 ENVELOPE，無釘住、$z_{hi}$ 只含 $h_{ceil}$ 與 $h_{\max}$，$z_{lo}$ 以指定超集帶之 $z_g^{\max}$；輸出 $z_{loc}=\mathrm{clip}(z_{des},z^*,u)$ 與 $h_{\min,l}=\min_{j\in[u_{lo},u_{hi}]}(z_{loc}(j)-z_g^{\max}(j))$（只在條帶本身取 min；$\pm R_{eff}$ 延伸段只提供爬降空間，胞外山丘不縮窄條帶）、$AGL_{\max,l}$、`feasible`（$z^*\le u$；不可行時該條帶／方向回報 §11.5）。

### 11.4 全架次剖面與寬度不變量

```
BUILD_3D(sortie2d, c) -> waypoints:
  samples = resample(all segments, d_d, arc = min(d_d, RΔψ))；ds[] 逐段
  compute z_lo, z_hi, z_des, tan_c, tan_d (§11.1；先以 pin_start/pin_end 之上界線定 CLIMB/DESCENT 段界 j_c, j_d)
  z*, u = ENVELOPE(..., ds, pin_start = z_site + H_tc_cmd, pin_end = z_site(land) + H_tc_cmd)
  if INFEASIBLE: 依 §11.5 處置後重試
  z = clip(z_des, z*, u);  z, _ = ENVELOPE(z as z_lo, u, ...)          # 二次保證坡度可行（z ≥ z* 且 z* 可行 ⇒ 結果 ≤ u）
  平滑（論文 Eq.37/38 投影梯度下降）：for it in 1..N_it: z_j -= lr·(c1·sign(z_j − z_des_j) + 2c2(2z_j − z_{j−1} − z_{j+1})); z = clip(z, z*, u)
  再一次 ENVELOPE(z as z_lo, u) 夾持
  點刪減（論文 Eq.39 3D 共線 + 最大間距 L_max=500 m）：保留 phase 邊界、sensor_on 切換點、弧段端點
  verify: ∀ OPER strip l: min AGL ≥ h_min,l；max AGL ≤ h_max,c·k_GSD；坡度在限內；z ≤ z_hi；
          phase ∈ {CLIMB, ENTRY, OPER, TRANS, RTB, DESCENT, ORBIT, HOLD} ⇒ mode = CRUISE（無懸停，A7）
  return waypoints (x,y,z,t,V_a*, mode, phase, sensor_on, arc params)
```

**寬度不變量（D36）**：OPER 樣本的下界含 $z_g^{\max}{}_{w_l/2}+h_{\min,l}$（§11.1），而 $z\ge z^*\ge z_{lo}$ 在 clip、二次 ENVELOPE 與投影平滑（皆夾持於 $[z^*,u]$）下恆成立 ⇒ $AGL_{final}(j)=z(j)-z_g^{\max}{}_{w_l/2}(j)\ge h_{\min,l}$ ⇒ 足跡 $w_c(AGL)\ge w_c(h_{\min,l})=w_l$ ⇒ §7.7 成立。此不變量與局部包絡的 $z_{lo}$（超集帶）無關，不依賴「$z^*_{full}\ge z^*_{loc}$」（該論證不成立：全架次 OPER 用 $b_j=w/2$、局部用超集帶，$z_{lo,full}\le z_{lo,loc}$）。反例（修正前）：走廊上 30 m 高、100 m 長小丘，$z_{des}$ 250→280→250，平滑後丘頂 $z=263.2$、AGL 133 m $<h_{\min}=150$，足跡 186 m $<$ 210 m ⇒ 兩側各 12 m 缺口；修正後 $z^*$ 於丘頂 $\ge z_g^{\max}+150=280$，平滑無法低於。反向風險 AGL $>h_{\max,c}$（GSD 變粗）由 $u$ 檢出。可行性代價：$z_{lo}$ 抬高後 $z^*>u$（空域上限）的情形交 §11.5。

### 11.5 不可行處置（依序）

0. $z^*>u$ 在 CLIMB/DESCENT：站點走廊淨空問題（$P_{tc}$ 後 $(h_{\min fw}-H_{tc})/\tan\gamma^g$ 內有障礙或陡升地形）→ 改 $\theta_{tr}$（障礙最少方位，若側風允許）或改站點；不在 §8.4 語意內。
1. $z^*>u$ 僅在 TRANS/ENTRY：§8.4 延長轉移（螺旋）。
2. 在 OPER（山谷／稜線交替使 3° 追不上或空域上限壓迫）：(a) 該胞改用沿等高線候選方向（`slope_exceeds` 旗標）；(b) 以最陡處為界沿等高線再分割胞；(c) 相機允許 $k_{GSD}=1.2$ 並記錄品質；(d) 空域上限：$h_{op}\leftarrow\max(h_{\min fw},h_{op}-10)$ 重試 ≤3 次並回饋 §7.4 重鋪（兩趟法）；(e) 報告不可覆蓋子區。
3. $z_{lo}>z_{hi}$ 逐點（高度帶為空）：PRECHECK 已攔截；任務中出現則該胞標記不可覆蓋。

### 11.6 起飛走廊與爬升盤旋

```
TAKEOFF_CORRIDOR(H, c, wind, K, S_entry, z_entry, t0, entry_len = cache.entry 之路徑長) -> segments, S_exit:
  θ_tr = β (逆風；W<2 → 障礙最少方位)；H_tc_cmd = hi if not airspeed_validated else lo
  seg1 = vertical(H, z_H → z_H + H_ts), t = H_ts/v_z_up, P_h
  P_tc = H + d_tr·t̂(θ_tr)（d_tr 依 §3.4 公式）；seg2 = transition(H→P_tc, H_ts→H_tc_cmd 線性、旋翼輔助 D37), P_tr, t_tr_out
       淨空：走廊 P_0↔P_tc 半寬 r_site+b_obs，障礙頂+b_v 低於高度線 H_ts+(H_tc_lo−H_ts)·x/d_tr（淨空用下緣，D29）
  seg2b = FW_ACCEL(P_tc → P_acc = P_tc + d_acc·t̂(θ_tr), 等高 H_tc_cmd, t_acc, P_acc)      # §3.4
  S_acc = (P_acc, θ_tr, R_eff)                                                        # 進入路徑起始狀態（IDP 之 S_i）
  # 爬升需求以包絡直接算（不用「D_path·tan_g ≥ Δh」：未乘弧段 κ 且忽略途中地形）：
  對 AVOID(S_acc → S_entry) 取樣，做無釘住 ENVELOPE 的後向遞迴得 b[0]（含 CLIMB 段 z_lo、途中山丘之 h_minfw/h_clr 需求、弧段 κ）
  Δh_req = b[0] − (z_H + H_tc_cmd)
  if Δh_req ≤ 0: return [seg1, seg2, seg2b], S_exit = S_acc, n_lap = 0     # 進入段本身由 cache.entry 計，不在此重複
  # 爬升盤旋（地面參考，R_orb ≥ R_eff）
  C = choose_orbit_center(near P_acc, upwind side preferred, R_orb, K ⊕ b_trk, C2)
  T_lap = ∮ ds/V_g(θ)；Δh_lap = V_a sinγ_c(m) cosφ_max · T_lap        # 弧段爬升折減
  n_lap = ceil(Δh_req/Δh_lap)
  ϑ* = argmin_ϑ [t(orbit 至切點 ϑ) + t(AVOID(S_exit(ϑ) → S_entry))] s.t. ENVELOPE 自 S_exit(ϑ) 可行
  return [seg1, seg2, seg2b, join(S_acc→orbit), orbit×n_lap, exit(ϑ*)], S_exit(ϑ*)   # 之後的進入段由 entry_cost(S_exit) 負責
```
反例（舊判準）：1200 m 全弧進入路徑、$\Delta h=50$：$1200\cdot0.04246=51\ge50$ 判無需盤旋，包絡實得 47.9 m（$\kappa=0.94$）→ BUILD_3D 才 INFEASIBLE；途中距 $P_{tc}$ 1 km 之 80 m 小丘要求 $z\ge$ 地形+120，3° 只能爬到 $z_H+142$ ⇒ 缺 58 m。現以 $b[0]$ 一次算足。

數值：A 無風 $T_{lap}=58.4$ s／每圈 61.8 m（$2\pi R\sin3°\cos20°$）；$W=5$ 積分 60.9 s／64.4 m；B 無風 39.7 s／35.8 m、$W=5$ 41.9 s／37.8 m（皆為 $T_{lap}$／$\Delta h$）。$\Delta h=50$ m（$H_{tc}=100\to150$）直線需 954 m（無風）／1178 m（順風 5 m/s）；$\Delta h=60$（→160 m）1413 m；$\Delta h=100$ → 1.9 km。爬升時間 $\Delta h/(V_a\sin3°)$ 與風無關（A 1.125 m/s）。CLIMB 段（100→120 m AGL）於 $P_{tc}$ 後 471 m 內完成（A）。

### 11.7 降落走廊（home／備降點通用；D19）

```
LANDING_CORRIDOR(S_last, z_last, T, c, wind, K) -> segments:
  θ_app = β(wind at H_tc)；P_tc = T − d_tr·t̂(θ_app)               # 轉換起點在下風側，逆風進場
  P_dec = P_tc − d_acc·t̂(θ_app)；S_app = (P_dec, θ_app, R_eff)     # FW_DECEL 起點 = 返航路徑目標狀態（§8.7）
  z_tc = z_T + H_tc_cmd
  D_path = |AVOID(S_last → S_app)|；tan_d = tan γ_d^g(順風最大 V_g)（保守）
  if D_path·tan_d ≥ z_last − z_tc（並以前向遞迴 a[N] ≤ z_tc 精算）: path2d = AVOID(S_last→S_app)；z = ENVELOPE(pin_start=z_last, pin_end=z_tc)
  else: C = choose_orbit_center(downwind of T or overhead, R_orb, K)；Δh_lap_d = V_a sinγ_d cosφ_max T_lap（A ≈ 86 m/圈 @W=5）
        n_lap = ceil((a[N] − z_tc)/Δh_lap_d)；path2d = AVOID(S_last→orbit) + orbit×n_lap + exit→S_app
  seg_dec = FW_DECEL(P_dec→P_tc, 等高 H_tc_cmd, V_a,c→V_tr,cmd, t_acc, P_acc)
  seg_tr  = transition(P_tc→T, H_tc→H_ts 旋翼輔助（D37；不受 4° 限制）, 減速至懸停), P_tr, t_tr_in；淨空同 §11.6（下緣）
  seg_v   = vertical(T, H_ts→0, hover_descent_time(H_ts) = (H_ts−10)/v_z_dn + 10/0.5), P_h (+ t_hov_res)   # 與 §5.3 共用
  missed approach：任一點中止 → 沿 θ_app 以 γ_c 直線爬升至 z_tc+50 m → 回下降盤旋；能量檢查 E(P_tc) ≥ r·E_nom + e_MA（§5.5），否則改備降點
  return segments
```
數值：A $\Delta h=50$ m 下降距離依規則（順風最大 $V_g$ 保守）$50/(21.5\sin4°/26.5)=883$ m（無風 715、逆風 5 m/s 實際 550 m，不利用）；$\Delta h=60$ → 1060 m。降落總時間 $t_{acc}+t^{in}_{tr}+t_{dn}+t_{hov,res}$：A $7+25+70+15=117$ s（走廊段不含 $t_{acc}$ 時 110 s）、B $6+25+60+15=106$ s（100 s）。

### 11.8 站點檢查（每個 home／alternate）

淨空圓 $r_{site,c}$ 內無障礙且 $r_{site}\ge$ 翼展/2+2 m；$b_{obs,c}$ 內無高於 $H_{ts}$ 之障礙；轉換走廊（$P_{tc}\leftrightarrow T$，半寬 $r_{site}+b_{obs}$）內障礙頂 $+b_v$ 低於走廊高度線（下緣）；**CLIMB／DESCENT 走廊**：$P_{tc}$ 後（起飛側）與前（降落側）$(h_{\min fw}-H_{tc,lo})/\tan\gamma^g_c$（順風 5 m/s 時 A 942 m、B 724 m，取下緣保守；無風 A 764／B 573 m。B 之舊值 480 m 係誤用 A 的下緣 80 m，正確為 (100−70)/tan γ^g_c = 724 m，見 `altitude.climb_corridor_length_m`）內、半寬 $b_{trk}$ 之走廊淨空（地形＋$h_{clr}$、障礙頂＋$b_v$ 低於自 $H_{tc,lo}$ 起以 $\gamma^g_c$ 爬升之高度線）；下降盤旋碟形區（$R_{orb}+b_{trk}$）無 keep-out、在 C2 內；站點風（側風 $\le X_{\max}$、陣風 $\le G_{site}$）；風向變化 $>45°$ 時重算走廊（§12.5）。

### 11.9 高度分層

A/B 作業層由感測需求決定；分層可行判定 $|h_{op,A}-h_{op,B}|\ge\Delta h_{sep}+\delta_{prof}$（$\Delta h_{sep}=50$ m、$\delta_{prof}=10$ m 剖面容差：平滑、走廊 DEM 差異、包絡抬升）且兩層在該區不重疊（比較同一 $(x,y)$ 之 $z$）→ 允許同胞（`both`）或相鄰胞同時作業。不足時 `layering()` 依序嘗試在各自高度帶內調整：提高 $h_{op,A}$（至 $h_{hi,A}$）→ 降低 $h_{op,B}$（至 $h_{lo,B}$）→ 反向；調整後對應構型重鋪條帶（§7.4）；仍不可行 → 時間分離（§13）。BUILD_3D 後以實際剖面驗證同區最小垂直間距 $\ge\Delta h_{sep}$（後驗）。示例：B 100 m、A 預設 150 m → 50 m 恰等於門檻，1 m 偏差即衝突 → A 調至 160 m（契約 §W 直接指定 160）。過境層 $h_{transit,c}=h_{\min fw,c}+h^{extra}_{transit}\cdot\mathbb 1[c=A]$（`cfg.h_transit_offset_A_m`=50；受 $h_{ceil}$ 限制）；起降走廊高度 $\le H_{tc}$ 與作業層分離。預設 $h_{ceil}=120$ 且無豁免時 A 不可指派（§4.6），分層自動退化為 B 單層。

---

## 12 動態重規劃

### 12.1 通用框架（論文 4.3 擴充）

```
REPLAN(event, world, plans, t_now) -> MissionPlan':
  1 快照：每機 S_i(t_now)、E^rem_i（遙測依 §5.6）、現行條帶 l_i^cur 與已掃比例；FOV 有效者先完成該條（論文 4.3.2）
  2 L_uncov ← 未覆蓋作業路徑 ∪ 中斷條帶自最後有效覆蓋點裁短之殘段（保留 e_ext）
  3 受影響集合 A = {i : κ(σ,c(i))=1 for some σ ∈ L_uncov 且 E^rem_i − e(剩餘含 RTB) ≥ r·E_nom + margin(5% E_nom)}
    （不限於退出機所在團隊——單機團隊為常態，否則 A=∅ 使 leftover 無人可投標）
    凍結語意 = 「不重排既有序列，但允許在序列末端（RTB 之前）追加插入」；拍賣以 Δmakespan 決定（§9.4 Phase 2）
  4 重算受影響快取層（風：時間層；R_eff>5%：幾何層；新胞：add_strips 新條帶列/行）
  5 ALLOCATE(leftover ∪ 受影響條帶, A, 初始 = 快照, t^avail_i = 完成現行條帶時刻, frozen = 既有序列) → BEAM_IDP(暖啟動 = 現行序列；事件驅動使各機進度不等自然處理) → BUILD_3D → DECONFLICT(僅變更機)
  6 ENERGY_SAFE_CHECK、鏈路、時間窗驗證；輸出 diff（只下傳變更機自 l_i^cur 之後的航點）
  時限 ≤ 2 s；超時先下發佔位計畫「續飛現行條帶 + 保守 RTB（預載 AVOID 至最近降落點）」，背景完成後再下傳
```

### 12.2 UAV 退出（UAV_EXIT）

`fleet.remove(i)`；其未完成條帶（含殘段）→ L_uncov；若 $c(i)=A$ 且無其他 A：lidar/both 條帶 → UNCOVERABLE 回報；rgb/any → 允許 B 接手（殘區以 B 條帶重鋪）；同構型接手保留原條帶（接手者 ∈ 受影響集合 A，§12.1 步 3，於其序列尾端追加）；REPLAN。

### 12.3 新區域（NEW_AREA）

DECOMPOSE 新區 → PRECHECK (7)(8)(9) 對新胞；候選機 = 可行構型且 $E^{rem}-e(\text{剩餘})-e(\text{插入})\ge rE_{nom}+e^{alt}$；拍賣插入（只接受 makespan 增量最小之 ≤2 團隊變更；正在作業的機先完成當前作業路徑，論文 Fig.15 策略）；否則多架次或 UNCOVERABLE(energy) 待辦。

### 12.4 能量偏差／低電量（ENERGY_UPDATE）

```
on ENERGY_UPDATE(i, E_tel):
  δ = (E_pred − E_tel)/E_pred
  if δ > δ_1 = 0.05: κ_i ← clip(κ_i·E_used,act/E_used,plan, 0.8, 1.5)；重算剩餘計畫能量（EnergyModel.correction）
  E_ref = E_pred(κ_i 修正後) if not battery_monitor_validated else clip(E_tel, 0.9·E_pred, 1.1·E_pred)   # §5.6：未驗證時遙測只用於 δ 與 κ
  if E_ref − e(剩餘含 RTB) < r·E_nom: cut ← 自尾端逐條移除作業路徑至可行；移除者 → L_uncov → REPLAN
  if E_ref < r·E_nom + e^alt(S_i, nearest) + margin(5% E_nom): 立即 DIVERT 至最近備降點；全部剩餘 → L_uncov
  if not battery_monitor_validated and δ > δ_2 = 0.15: 保守 DIVERT
```

### 12.5 失鏈（LINK_LOSS；D26）

每機預載策略 `lost_link_policy ∈ {RTB, CONTINUE_THEN_RTB, NEAREST_ALT}`（預設 A：CONTINUE 至 $t_{ll}=60$ s 後 RTB；B：RTB）；每航點附 `lost_link_action` 與 `nearest_alternate`。地面：以預載動作推算該機 4D 航跡（$\pm\sigma_t$，$k_t=0.05$）並登記為「幽靈航跡」，其他機在 §13 中對其避讓、其目標降落走廊預約 $[ETA-60, ETA+180]$ s；$t-t_{loss}>t_{ll}$ → 視同 UAV_EXIT（保留幽靈走廊獨占）；恢復 → ENERGY_UPDATE 後重新加入拍賣。

### 12.6 風場更新（WIND_UPDATE）

重算 $V_g,t,e$（含 RTB，時間層）；$R_{eff}$ 變化 >5% → 轉移幾何層重生（作業路徑不變）；任一機 ENERGY_SAFE_CHECK 失敗 → §12.4 cut；$\max\Delta C_i>\tau_w=5\%$ 或方向候選總時間變化 >10% → 未開始之胞重跑方向選擇與 IDP，已開始之胞只重跑剩餘（保留條帶）；起降風向變化 $>45°$ → 重算走廊；任一層 $W>W_{abort}$ → WEATHER_ABORT；地面站風超轉換限 → 延後降落（上風側 orbit，需能量）或改備降點。

### 12.7 目標線索 → 近距確認（TARGET_CUE, cue-and-confirm）

```
on TARGET_CUE(p, src=i_A, conf):
  Q_conf = 以 p 為中心、半徑 r_conf=150 m 之圓 → 8 邊形；σ=rgb；val=val_high；時間窗 [t_now, t_now+t_conf_max=15 min]
  GSD_req = gsd_confirm（預設 0.025 m ≥ h_minfw,B·p/f = 2.44 cm；舊值 2 cm 對應 h_max=82 m < 100 m，clip 後實得 2.44 cm 卻靜默通過）
  if h_max,B(GSD_req) < h_minfw,B: 回報 gsd_unattainable(GSD_req, 可達 h_minfw·p/f)；confirm_mode ← "orbit"（以最低高度盤旋、記錄實得 GSD）
  else: h_op^B = clip(h_max,B(GSD_req), h_minfw, h_ceil)
  候選 = {j ∈ U_B : Q_conf ⊕ d_turn ⊂ P_link,B, 插入後能量可行, C_j 增量最小}
  if 候選 ≠ ∅: 拍賣插入（可插在當前條帶之後、優先於低 val 胞；makespan 增量 ≤ τ_cue=10%）；
       確認路徑 = 單條帶（直徑 ≤ s_B）或 2–3 條帶（單向可行時以同向轉場 D34），或 confirm_mode="orbit"（R_eff, 2 圈）
  elif "rgb" ∈ available_kinds(A) and 某 A 能量允許: A 自行降至 max(h_min^A, h_GSD(A 之 RGB 硬體)) 確認（GSD 受 120 m 限制，記錄）
  else: 待辦佇列（priority）；A 不中斷 LiDAR 掃描
```

### 12.8 優先度變更（PRIORITY_CHANGE）

$val_k\leftarrow val'$；重算 $J$（僅 $\omega_4$ 項）；未開始之胞允許拍賣搶占（能量可行）；進行中且 $val$ 降低 → 不中斷。

### 12.9 氣象中止（WEATHER_ABORT；D23）

```
on WEATHER_ABORT(scope ∈ {ALL, config c, region}, reason):
  受影響機：若剩餘現行條帶 < 60 s 且能量允許則完成，否則立即中止
  OK = {a ∈ ALT ∪ {H} : 該點側風／陣風 ≤ 轉換限}
  降落點 = argmin_{a∈OK} t(S_i→a)；若 OK = ∅（全區大風，氣象中止之常態）: 降落點 = argmin_a [側風超限量]，warnings += landing_over_limit（仍優於不落）
  走廊排程：同一降落點依「能量時間餘裕 (E^rem − r·E_nom − e^alt)/P_c」升序（最緊者先），ETA 僅為 tie-break；間隔 t_sep,LDG=90 s；
           後續機於上風側 orbit 等待前先做能量檢查（每圈 A ≈16 Wh），不足 → 插隊（餘裕排序自動成立）或次近備降點
  未覆蓋 → 待辦；氣象恢復（觀測回到放飛門檻連續 10 min）後 REPLAN
```

---

## 13 防撞／去衝突

1. **高度分層**（§11.9，含 $\delta_{prof}$）為第一道；同層才做下列處置。
2. **同層空間互斥**（優先於時間分離）：同構型各團隊的胞多邊形 $\oplus$ 迴轉帶（D38）互不相交 → 幾何上不可能衝突，免做 4D 檢查；相交者才進入時間分離。
3. **4D 分離**：每機航點依時間戳線性內插，取樣 $\Delta t=2$ s，時間不確定度 $\sigma_t=k_t(t-t_{sync})$（$t_{sync}$ = 最近一次遙測快照時刻；初始規劃 $t_{sync}=t^0$）；以 `cKDTree` 於 $(x,y,V_{ref}t)$（$V_{ref}=20$ m/s）查詢，違規 $\iff$ 水平 $<d_{sep}=100$ m 且垂直 $<\Delta h_{sep}=50$ m 且 $|t_a-t_b|<\Delta t+\sigma_{t,a}+\sigma_{t,b}$；$O(N\log N)$。
4. **同胞多機**：IDP 剪枝（§10.2：迴轉帶時間重疊）＋同路徑不對飛＋側向間距 $s\ge d_{sep,lat}=60$ m（A 126–168、B 103–105 ✓）＋ skip-$k$ 使同時佔用條帶相距 $\ge ks$。
5. **起降走廊互斥**：`CorridorSchedule` 區間表；起飛間隔 60 s；降落預約 $[t_{arr}(P_{tc})-30,\ t_{land}+t_{sep,LDG}]$（$t_{arr}$ = 抵達降落走廊 $P_{tc}$ 時刻、$t_{land}$ = 觸地時刻，$t_{sep,LDG}=90$ s）；降落優先於起飛；衝突 → 在保持點（走廊入口外 2 km、過境高度）盤旋等待（能量檢查後允許，否則改備降點）；備降點被佔用 → 次近。
6. **修復（一次算足）**：需求延遲 $\Delta^{req}=\Delta t+\sigma_{t,a}+\sigma_{t,b}-|t_a-t_b|$（$t\approx1500$–1800 s 時 $\sigma$ 達 75–90 s、$\Delta^{req}$ 可達 150–180 s，逐圈試探 3 次不足）。依序：(i) 起飛延遲 $+\Delta^{req}$ 或起飛換序（零能量；需重排走廊）；(ii) 優先級低者（makespan 貢獻小）於最近前一轉移插入 $n=\lceil\Delta^{req}/T_{lap}\rceil$ 圈延遲盤旋（不用懸停），重驗能量；(iii) 仍衝突 → IDP 重排該階段（交換條帶方向／改同向轉場為零成本選項）；$\le3$ 輪。不採 ±10/15 m 垂直偏置（D18）。

---

## 14 複雜度與即時性

情境 $n_U=8$、$n_Z=3$、條帶 $n\approx80$、胞 $n_C\approx8$、keep-out 頂點 $V\approx100$、DEM $10^6$ 節點、$\lambda\le4$。

| 階段 | 複雜度 | 估計（Python+numpy/shapely） |
|---|---|---|
| PRECHECK + ENU | $O(\text{點數})$ | 5 ms |
| DEM 重取樣＋障礙燒入 | $O(\text{grid})$ 一次性 | 150 ms（初始化） |
| 凸分割＋合併 | $O(r n_v^2)$ | 20 ms |
| 方向候選＋條帶（含局部包絡，前 3 名精算） | $O(n_C\cdot3\cdot n_l\cdot N_s)$ | 40 ms |
| 切線圖／可視圖 | $O(V^2\log V)$ 一次性 | 30 ms |
| 成本快取：op $2n$、tr $4n^2$ Dubins（向量化）、entry $2n_Un$、rtb；EnergyBudgetTable $2n|\mathcal A\cup H|$ 次 AVOID+ENVELOPE（一次性） | $O(n^2)+O(n|\mathcal A|N_s)$ | 60 ms + 80 ms |
| 配置＋拍賣（含殘帶候選對）＋局部改善 | $O(|C|^2n_U\bar L)$ | 25 ms |
| 事件驅動 Beam-IDP（D33）：階段 $=n$，每階段 $\le B\cdot K$ 子節點 | $O(nBK)$；$K=6,B=3,n=80$ → 每階段 $\le18$、共 $\approx1.5\times10^3$ 節點（向量化 `stage_cost`） | 20–50 ms（向量化）；純 Python 逐候選迴圈 ≈0.1 s |
| `legacy_mode` 鎖步 IDP（論文結構） | $O(\frac n\lambda BK^\lambda)$；$\lambda=4$ → 每階段 3888、共 $\approx8\times10^4$ 節點 | 向量化 0.3–0.5 s；純 Python 逐排列迴圈 ≥2 s（超出即時預算，僅供退化對照） |
| RTB＋能量修復（截斷＋暖啟動 IDP） | $O(n_U|\mathcal A|)+O(nBK)$ | 30 ms |
| 高度剖面（每機 ~3000 點） | $O(N)$（兩趟＋$N_{it}N$ 平滑） | 60 ms |
| 4D 去衝突（先空間互斥再 KD-tree） | $O(N\log N)$ | 40 ms |
| **合計（重規劃路徑，初始化除外）** | | **≈0.4–0.6 s** |

論文完整列舉 $A^\lambda_n$：$n=27,\lambda=4$ → $4\times10^5$／階段（不可即時）；$n=60,\lambda=4$ → $1.2\times10^7$。鎖步＋k-nearest 降至 $K^\lambda=1296$；事件驅動再降至 $K$。即時策略：構型鍵快取、幾何／時間層分離、`realtime_budget_s=1.0` 超時 $B\to1$、增量重規劃（凍結未受影響團隊、暖啟動、diff 下傳）、能量預算表 $O(1)$ 剪枝、覆蓋柵格只在驗證執行、熱路徑以候選矩陣向量化（`IDPNode` 陣列欄位，無逐候選 Python 呼叫）。`runtime_under_1s` 測試排除 DEM 初始化（150 ms）。

---

## 15 與論文對照（含同構退化）

| 論文元件 | 本規格 | 退化條件 |
|---|---|---|
| Eq.1 定速運動學、$R_{\min}$、$\gamma$ | §3.2 風三角、$V_a^*$、$R_{eff}$、$\gamma_c\ne\gamma_d$、AUW 修正 | $W=0$、$m=m_{ref}$、$\gamma_c=\gamma_d$ |
| Eq.2–3 梯形 FOV | §4 LiDAR／相機（保守 AGL）；梯形選項（§4.3 明寫斜距投影公式） | `fov_model="trapezoid"`、常數 $h$ |
| Eq.4 DEM 雙線性 | `map_coordinates(order=1)`；走廊 max/min | 中心線取樣 |
| DP 簡化＋整體凸化 | §7.2 只外擴 DP＋凸分割（近凸即接受、保證回傳）＋合併 | `cfg.convexify_whole=True` |
| $\alpha_k$ 最小寬度方向 | §7.3 候選（含 $\beta,\beta+\pi$）＋$T_{proxy}$（風、轉場、坡度、$\Delta z$、迴轉帶） | $W=0$、平地 → 最小寬度 |
| Eq.5–9 條帶與端點 | §7.4 變寬序列鋪設、置中平移、$e_{ext}$、裁剪極值 | 常數 $w$、$e_{ext}=0$ |
| Eq.10–12 二元距離 | §8.2 二元時間、四狀態明確、轉移 4 型（D34） | 除以常數 $V$、只保留反向 2 型 |
| Eq.13–26 矩陣與限制 | 元素改時間、$\infty$ 不可行；Eq.(25) 放寬為「作業方向 = 轉移終端方向」＋能量／鏈路／感測硬限制、RTB | 限制關閉、Eq.(25) 嚴格交替 |
| Eq.27–29 繞行／離散 | §8.5–8.6 切線圖、圓角偏移；離散以 $e_{\max}$ | 單圓威脅、$\Delta\psi=\psi_{\max}$ |
| Eq.30–31 $J_k,\lambda_k$ | §9.2–9.3 六項時間版、每構型獨立、能量下限（部分覆蓋） | $\omega_5=\omega_6=0$、單構型 |
| Algorithm 1 | §9.4 貪婪＋拍賣（含殘帶候選對）＋局部改善＋BALANCE＋多架次 | 跳過 Phase 2–5 |
| Eq.32–34 IDP | §10 事件驅動 Beam-IDP（D33）、min-max 階段成本、逐機能量剪枝、返航（先 $H$）、跨胞 | 鎖步、$B=1,K=\infty$，`metric="paper"` |
| Eq.35 曲率補償 | ECEF→ENU 精確等價（50 km 下垂 196 m） | 平面近似時啟用 Eq.35 |
| Eq.36 高度下降 | §11.2 雙向包絡（最低可行剖面）；Eq.36 為單向貪婪近似 | — |
| Eq.37–39 平滑／刪減 | §11.4 投影 GD（不穿包絡）、3D 共線＋$L_{\max}$ | 同 |
| 起降／能量／鏈路／氣象／防撞 | §11.6–11.8、§5、§6、§13 新增 | 略去 → 論文起始狀態 |
| 動態事件（退出／新區） | §12 八類 | 僅前兩類 |

`legacy_mode=True` 一鍵設定全部退化條件（明列改寫欄位：`idp_branching="lockstep"`、`transfer_types=("rev",)`、`B=1`、`K=inf`、`metric="paper"`、`convexify_whole=True`、`fov_model="trapezoid"`、`e_ext=0`、`k_bm=k_as=1.0`、能量／鏈路／時間窗限制關閉、$t^{RTB}=0$、$h_{ceil}=\infty$）；驗證 §17 (7)：論文 Table 2 情境（3 機、$V=230$、$R=4000$、$h=1200$、$w=2200$、平地）路徑長應落在 590–650 km。**同構退化敘述**：兩構型參數相同、$W=0$、能量與鏈路限制設為無限、$h_{ceil}=\infty$、單胞、`metric="paper"`、$B=1$、$K=\infty$、鎖步分支時，§7–§11 逐式等於論文 Eq.(5)–(39) 與 Algorithm 1–3（唯高度為雙向包絡，於地形平緩時與 Eq.36 給出相同剖面）。

---

## 16 參數表

### 16.1 表 1 全部鍵（值不變；見 §3.1）

來源 T1；`provisional` 集合 = {vtol_site_clear_radius_m, obstacle_horizontal_buffer_m, transition_start_agl_m, transition_complete_agl_m, max_airspeed_mps(B), payload_max_kg(B), c2_link_range_km, terrain_clearance_m}；`provisional_supplementary` = §3.5 全部鍵。

### 16.2 演算法／模型參數（來源：D=本規格預設、M=任務輸入、prov=provisional）

| 參數 | 預設 | 單位 | 來源 | 用途 |
|---|---|---|---|---|
| battery_nominal_wh (A/B) | None（**示例** 1500/420，使用時警告 `battery_nominal_example`） | Wh | M | §5、§5.7 |
| reference_battery_wh (A/B) | None → = nominal 並警告 | Wh | D(prov) | D39、§5.1 |
| reference_test_depth_of_discharge f_dep | 1.0 | — | D(prov) | §5.1 |
| hover_power_ratio k_h, η_p, η_idle, p_accel_ratio | 4.0, 0.6, 0.3, 1.2 | — | D(prov) | §5.1 |
| t_tr_out / t_tr_in / t_hov_res | 20 / 25 / 15 | s | D(prov) | §5.3 |
| v_z_up / v_z_dn | 2.0 / 1.0（最後 10 m 0.5） | m/s | D(prov) | §5.3, §11.6–7 |
| a_tr, t_hold, transition_ground_run_m, gamma_tr_max_deg, transition_profile | 0.5 m/s², 8 s, None（公式 468/384 未驗證；384/308 驗證）, 8°, "rotor_assisted" | | D(prov) | §3.4 |
| dV_tr_margin | 2.0 | m/s | D | D8 |
| k_bm, k_as（PlannerConfig；驗證旗標為 true 時取 1.0） | 1.15, 1.05 | — | D | §5.2 |
| temp_cap_coef_per_c, temp_cap_floor | 0.01, 0.8 | | D | §5.2 |
| k_gamma_auw | 2.0 | — | D | D21 |
| lidar hw: fov_usable_deg, point_rate_hz, eta_ret, max_range_m, k_rng | 70, 600e3, 0.5, 450, 0.9（**示例值，待填入**；`is_default_example`） | — | M | §4.1 |
| camera hw: f, S_w, S_h, pitch, max_shutter | 16, 23.5, 15.6 mm, 3.9 µm, 1 Hz（**示例值，待填入**） | — | M | §4.2 |
| PayloadRequirement: density_pt_m2, gsd_req, o_side (lidar/cam), o_fwd, shutter_rate_hz, h_target | 20, 0.05 m, 0.25 / 0.30, 0.70, ≤ 硬體, A 150 / B min(h_max/1.2, h_max−20)≈171 | | M(D) | §4, §4.6 |
| t_settle | 2.0 | s | D | §4.5 |
| v_g_min_mps | 3.0 | m/s | D | §3.2 |
| r_eff_wind_policy | "max_layer" | — | D | §3.2 |
| W_go, G_go, vis_go, precip_go, T_range | min(8,0.45V_a), 12, 3000, 0.5, (−10,40) | m/s, m, mm/h, °C | D | §6.2 |
| BEAUFORT_UPPER_MPS | {5: 10.7, 6: 13.8, …} | m/s | D | §6.2 步 2 |
| W_abort, G_abort, vis_abort, precip_abort | 10, 14, 1500, 2.0 | | D | §12.9 |
| crosswind_site_max (unval/val) | A 6/8, B 5/6 | m/s | D(prov) | §3.4 |
| gust_site_max (unval/val) | 8/10 | m/s | D(prov) | §3.4 |
| site_max_slope_deg | 5.0 | ° | D | §6.2 步 5 |
| max_agl_m / waiver_max_agl_m | None（皆缺 → 120 並警告）/ None | m AGL | M | §4.6 |
| dh_band_min | 20 | m | D | §4.6 |
| terrain_clearance_m | 必填（建議 ≥3σ_DEM+30） | m | M | §11.1 |
| c2_link_range_km（逐構型） | 必填 | km | M | H2 |
| platform_overrides | {} | | M | §3.5 |
| b_trk, b_h, b_v | 50, 50, 30 | m | D | §7–8, §11 |
| δ_edge = k_σ·σ_xy + e_xt + δ_w/2（k_sigma, sigma_xy_m, e_xt_m） | 15（2.0, 3.0, 5.0 → 11 + δ_w/2） | m | D | §7.2 |
| ε_dp factor, τ_merge factor | 0.1·min s, 0.5·min w | — | D | §7.2 |
| k_s (slope pen), λ_cut, λ_n, convexify_whole, widen_on_lift | 20, 1e-3, 30, False, False | — | D | §7.2–7.4 |
| e_max, ψ_max, Δθ_int | 3 m, 30°, 5° | | D | §8.1, §8.5 |
| r_acc | min(R_eff/2, 30) | m | D | §8.5 |
| transfer_types | ("rev", "same")（legacy: ("rev",)） | — | D | §8.2 |
| ν (S 權重) | (0.15,0.15,0.20,0.20,0.20,0.10) | — | D | §7.6 |
| ω (J 權重) | (0.2,0.15,0.2,0.15,0.1,0.2) | — | D | §9.2 |
| κ_load, N_ls | 0.85, 200 | — | D | §9 |
| t_swap, multi_sortie, recovery_site | 600 s, False, home | | D | §9.6, D40 |
| Δt_dep, t_sep_LDG | 60, 90 | s | D | §9.7, §13 |
| idp_branching, K, B, ε, μ, τ_0 | "event", 6, 3, 0.05, 0.0, 60 s | | D | §10 |
| realtime_budget_s | 1.0 | s | D | §10.4 |
| d_d, N_it, c1, c2, lr, eps_col, L_max | 25 m, 50, 1.0, 0.5, 0.2, 1e-3, 500 m | | D | §11 |
| k_GSD, k_gsd_relax | 1.0, 1.2 | — | D | §11.5 |
| h_transit_offset_A_m | 50 | m | D | §11.9 |
| Δh_sep, δ_prof, d_sep, d_sep_lat, k_t, Δt_4d | 50, 10, 100, 60, 0.05, 2 | m, m, m, m, —, s | D | §11.9, §13 |
| lost_link_policy, t_ll | A CONTINUE_THEN_RTB / B RTB, 60 s | | M(D) | §12.5 |
| δ_1, δ_2, energy margin | 0.05, 0.15, 0.05 E_nom | — | D | §12.4 |
| r_conf, gsd_confirm, t_conf_max, τ_cue, confirm_mode | 150 m, **0.025 m**, 900 s, 0.10, "strips" | | D | §12.7 |
| τ_w | 0.05 | — | D | §12.6 |
| cov_raster_m | 5 | m | D | §17 |
| dem_step_m | 10 | m | D | §6.2 |

---

## 17 驗證計畫

### 17.1 驗證項目（sim.py）

1. **覆蓋完整性**：每區 5 m 柵格；每架每 `sensor_on` 航段依實際 AGL 算足跡（相機另算前向重疊）→ 掃過多邊形 union；覆蓋率 ≥99.5%（`both` 區兩感測器各 ≥99.5%）；輸出未覆蓋格 GeoJSON。
2. **最小分離**：§13 檢查器重跑；回報最小水平／垂直距離與時刻；走廊排程無重疊。
3. **坡度／AGL／曲率／模式**：固定翼段相鄰航點坡度 $\in[-\tan(\texttt{max\_descent\_angle\_deg}),\tan(\texttt{max\_climb\_angle\_deg})]$（max 值驗、planning 值規劃；轉換段以 $\gamma_{tr,\max}$ 驗）；AGL $\ge h_{clr}$ 於 $d_d$ 取樣；CLIMB/DESCENT 以外的固定翼段 AGL $\ge h_{\min fw}$；OPER 段 AGL $\ge h_{\min,l}$ 且 $\le h_{\max}k_{GSD}$；$z\le z_{hi}$；離散折線曲率 $\ge R_{eff}$；`mode==HOVER ⇒ phase ∈ {TAKEOFF, LAND}`、固定翼 phase ⇒ `mode==CRUISE` 且 `airspeed ≥ v_min_mission`（A7 無懸停轉向）。
4. **能量與備降**：逐航點（至 $P_{tc}$）$E^{rem}\ge rE_{nom}+e^{alt}$（以 EnergyBudgetTable 真實值）；$E(P_{tc})\ge rE_{nom}+e_{MA}$；落地餘量 $\ge rE_{nom}$；$D^{plan}\le D^{use}$；第一架次落點 $=H$（除非 `planned_landing_at_alternate`）。
5. **鏈路／半徑／時間窗／禁區**：所有航點（含迴轉帶掃掠足跡 ⊕ $b_{trk}$）在各構型覆蓋內、$\le\rho^{sym}_c$、在窗內、不入有效禁區。
6. **Makespan 比較**：異構 vs 同構基線（全 A／全 B）；`legacy_mode` 於同構機隊 vs 本演算法；報告 makespan、總距離、計算時間。
7. **論文情境重現**：Table 2（3 機、Z1 六頂點、$V=230$、$R=4000$、$h=1200$、$w=2200$）平地 `legacy_mode` → 路徑長 590–650 km。
8. **閉式對照**：Ω、U、環型長度與 6-word 求解器一致（相對誤差 <1e-9）；$W=5$ m/s 時 A 圓周時間因子 1.043。
9. **運動學追蹤模擬**（sim.py）：以 L1/純追蹤模型沿航點飛行（含風），驗證橫向偏差 $\le e_{\max}+e_{xt}$、能量積分與計畫偏差 <5%。

### 17.2 pytest 清單

```
tests/test_geo.py            enu_roundtrip_mm；curvature_drop_50km(≈196 m)；bilinear_matches_eq4；corridor_max_terrain；enu_z_msl_roundtrip
tests/test_platform.py       table1_keys_exact；table1_values（僅 PLATFORM_* 常數）；climb_rate_consistency；bank_radius_consistency；R_eff_trochoid(A,W=8→245)；
                             W_star(5.17/5.03)；gamma_c_auw；stall_margin；transition_cmd_unvalidated（d_tr 468/384）；hero_overrides（300 覆寫、(5,6)、payload 1.0 prov）；
                             range_fraction_consistency；payload_check（估算＋NO_GO）；beaufort_upper_table；platform_overrides_only_provisional
tests/test_sensor.py         lidar_swath_speed_limit；camera_gsd_alt_speed（h_max=205）；trapezoid_matches_paper（w≈2200.1,l≈999.8，除以 sin）；
                             conservative_agl_width_monotone；gsd_unattainable_flag（2 cm → 82 m < 100）；requirement_overrides_hardware；default_sensor_warning
tests/test_energy.py         usable_distance_cap（模型值 A 120/145、B 81/97 km 對表 133–145／79–86：未驗證值落在表內或以下，驗證值超表時 cap 生效）；auw_scaling_1p5；reserve_deducted_once；
                             energy_safe_check_intermediate_points（止於 P_tc）；base_minute_mode（僅 soc）；pack_size_invariant_power；
                             landing_10m_rule（A 70 s、79.7/81.2 Wh）；transfer_energy_climb_power；budget_table_terrain_detour（谷地→稜線 +42 Wh）
tests/test_mission.py        table2_fields_present；precheck_missing_c2_is_nogo（逐構型）；precheck_wh_without_nominal_is_nogo；precheck_weather_go_nogo；
                             A_needs_waiver_flag；dsm_skips_tree_building_burn；time_windows_three_kinds；geojson_multipolygon_split；
                             site_agl_altitude；wind_layer_agl_ref；airspace_ceiling_default_warning；battery_temp_fallback
tests/test_wind.py           ground_speed_triangle（(2,) 與 (N,2) 廣播）；crosswind_infeasible；arc_time_jensen（circle factor ≥1）；r_eff_wind_policy_max_layer
tests/test_dubins.py         six_words_vs_bruteforce；omega_closed_form；loop_closed_form(3πR+2R−s)；uturn_closed_form；
                             transfer_state_pairing_four_types（只斷言長度不斷言 word）；discretize_circumscribed_error≤e_max
tests/test_geometry.py       simplify_outward_contains_original；concave_L_two_convex；polygon_with_hole；merge_shallow_notch；
                             small_notch_accepted_near_convex（500 m 方形 20 m 凹口）；partition_always_returns；multipolygon_after_nofly；
                             strip_count_formula；lay_strips_matches_count（W=1270 → 8 條）；strips_union_covers_cell（後置斷言）；centered_shift_relay_no_gap；
                             superset_band_conservative；width_never_exceeds_superset（h_min > h_B）；sweep_origin_consistent；nofly_buffer_btrk
tests/test_decompose.py      direction_min_width_no_wind（退化）；direction_changes_with_wind；wind_axis_candidates_present；slope_penalty_finite；
                             delta_z_penalty_orbits；turn_band_omega_overshoot（A 487 m、B 288 m）；one_way_feasible_cell_proxy_finite；
                             altitude_band_polygon_level；residual_candidate_pairs
tests/test_costs.py          cost_matrix_shape_four（tr (n,n,4)）；index_convention_idx（tr[l,m,0] 終端方位 = θ_LR）；T_transfer_uses_idx_m；
                             wind_refresh_keeps_geometry；avoid_tangent_graph_clear；visibility_fallback_vertex_buffer；budget_table_o1；
                             static_filter_h4；entry_includes_orbit_laps；rtb_index_end_side；per_class_indices_add_remove
tests/test_allocate.py       sensor_hard_constraint（available_kinds）；energy_infeasible_not_assigned；lambda_bounds_energy_floor；energy_overhead_infeasible（denom ≤ 0）；
                             partial_coverage_when_short；auction_multi_cell；auction_residual_pairs；phase2_records_uncoverable；move_swap_hard_feasible；
                             local_search_reduces_makespan；time_window_delayed_takeoff；multi_sortie_first_lands_home；takeoff_order_by_C_hat
tests/test_planner.py        direction_equals_transfer_terminal(Eq.26)；each_strip_once(Eq.20–21)；beam1_paper_metric_equals_full_dp(n=6,λ=2, legacy lockstep)；
                             event_driven_beats_lockstep_unequal_strips（8 長+16 短：≤770 s vs 1220 s）；knearest_within_2pct(n=12,λ=2)；
                             per_uav_rtb_keeps_others（3 架 A、A3 能量不足）；same_direction_transfer_one_way_cell（B、GSD 2.5 cm、W=8）；
                             append_rtb_home_first；energy_repair_truncates_at_xj；cross_cell_chain_prefers_cell_completion；beam_sorted_by_W；
                             no_hover_outside_vtol_phases；stage_cost_vectorized_signature
tests/test_altitude.py       envelope_forward_descent_backward_climb（ds 陣列）；envelope_is_pointwise_infimum；envelope_slopes_within；arc_sampling_min_dd；
                             pin_end_upper_bound；climb_phase_below_minfw_feasible（平地無風每架次可行；CLIMB 471 m）；oper_lower_bound_includes_hmin（小丘反例 AGL ≥ 150）；
                             ridge_gt_3deg_flag；smoothing_then_clamp；takeoff_orbit_count_by_envelope(Δh=100,entry 500 m；1200 m 全弧 Δh=50 → 1 圈)；
                             orbit_gain_cos_phi（PIONEER (61.8, 58.4)；HERO (35.8, 39.7)）；landing_ptc_downwind；descent_dist_tailwind_conservative（883/1060 m）；
                             site_checks（含 CLIMB 走廊）；layering_margin_delta_prof；transition_rotor_assisted_slope；accel_segment
tests/test_dynamic.py        uav_exit_partial_strip；uav_exit_single_uav_team_affected_set；new_area_incremental；energy_deviation_kappa（未驗證用 E_pred）；link_loss_ghost_track；
                             wind_update_time_layer_only；cue_dispatches_B；cue_gsd_unattainable_orbit；priority_preempt；weather_abort_landing_order（能量餘裕）；weather_abort_all_over_limit
tests/test_deconflict.py     layering_A_above_B（160/100）；spatial_exclusion_skips_4d；detect_repair_one_shot_n_laps；repair_prefers_takeoff_delay；
                             corridor_reservation_semantics（P_tc−30, land+90）；same_cell_turn_band_pruned；sigma_t_since_sync
tests/test_sim.py            example_coverage_ge_99p5；all_checks_pass；runtime_under_1s（8 機/3 區/80 條帶，排除 DEM 初始化）；
                             legacy_reproduces_paper_table2；hetero_le_homogeneous_baseline；mini_example_numbers（module_contract §W.10）
```

---

## 18 審查回應（v1.1 修訂紀錄）

審查共 82 項（blocker 8、major 34、minor 40）。全部 blocker 與 major 已採納；minor 全部採納，其中 5 項以等價或更完整的方式處理，1 項的數值結論修正。逐項處置如下（依審查順序）。

| # | 嚴重度 | 章節 | 處置 |
|---|---|---|---|
| 1 | blocker | §11.1 z_lo × BUILD_3D 釘住 | 採納：新增 CLIMB／DESCENT 段（D35、§11.1）、§6.2 步 5／§11.8 CLIMB 走廊淨空、§11.5 第 0 項。 |
| 2 | blocker | §11.4 寬度不變量 | 採納：OPER 樣本 $z_{lo}$ 含 $z_g^{\max}{}_{w/2}+h_{\min,l}$（D36），論證重寫為 $z\ge z^*\ge z_{lo}$；反例列入 §11.4 與測試。 |
| 3 | major | §7.3 迴轉帶 × Ω | 採納：$d_{turn}$ 幾何（D38）用於 §7.3、§7.6 $s^{turn}$、H2、§6.2 裁切、§10.2／§13 剪枝。 |
| 4 | major | §7.4 迴圈條件 | 採納：`while strips==[] or strips[-1].v_hi < v_max`、$\Delta=$`strips[-1].v_hi − v_max`。 |
| 5 | major | §3.4 轉換段幾何 | 採納（方案 a 之變體，D37）：轉換段定義為旋翼輔助飛行（$P_{tr}=P_h+P_c$ 本就含懸停功率），高度變化不受固定翼 3°/4° 限制而受 $\gamma_{tr,\max}$；固定翼限制自／至 $P_{tc}$；`transition_profile="fw_slope"` 提供方案 b。淨空高度線幾何不變（物理解釋改變）。另依 minor 建議加 FW_ACCEL／FW_DECEL。 |
| 6 | major | §7.1 禁區緩衝 | 採納（D41）：$P\setminus\bigcup(\text{NoFly}\oplus b_{trk})$、胞外擴後 $\cap F$、作業路徑逐條檢查。 |
| 7 | major | §4.6 $h^{target}_B$ | 採納：預設 $\min(h_{\max}/1.2,h_{\max}-20)\approx171$ m；GSD 可達性檢查（§4.2）。 |
| 8 | major | §7.2 $A_{\min}$ | 採納：近凸即接受、$A_{\min}\to0$ 退階、保證回傳、MultiPolygon 逐一處理。 |
| 9 | major | §13 4D 分離 × 修復 | 採納三項：一次算足 $n$ 圈／起飛延遲優先、同層空間互斥、迴轉帶時間重疊剪枝；另把 $\sigma_t$ 改為自最近快照起算。 |
| 10 | major | §7.4 $w=w_c(h_{\min})$ | 採納：$w=w_c(\min(h_{\min},h_B))$，`widen_on_lift` 選項。 |
| 11 | major | §12.7 gsd_confirm | 採納：預設 0.025 m、可達性檢查、不可達 → orbit 並回報。 |
| 12 | minor | §11.6 $n_{lap}$ | 採納：以無釘住 ENVELOPE 後向遞迴 $b[0]$ 定 $\Delta h_{req}$；降落側前向 $a[N]$。 |
| 13 | minor | §7.3 $T_{proxy}$ $\Delta z$ 項 | 採納：$\text{pen}_{\Delta z}$ 與 $s^{turn}$。 |
| 14 | minor | §11.2 $ds$ 陣列 | 採納：ENVELOPE 接受 `ds[]`，弧段取樣 $\min(d_d,R\Delta\psi)$。 |
| 15 | minor | §7.4 $h_{\min}$ 範圍 | 採納：只在 $[u_{lo},u_{hi}]$ 取 min。 |
| 16 | minor | §11.7 數值 | 採納：統一為順風保守值（883 m @50 m、1060 m @60 m）。 |
| 17 | minor | §8.9 entry 與盤旋 | 採納：暫定入口先跑走廊、`entry` 含 $n_{lap}T_{lap}$、`refresh_entry`。 |
| 18 | minor | §11.9 分層恰 50 m | 採納：$\Delta h_{sep}+\delta_{prof}$（10 m）、`layering()` 可調整 $h_{op}$；示例 A 改 160 m（契約 §W 全部重算）。 |
| 19 | minor | §7.3 候選方向 | 採納：加入 $\beta,\beta+\pi$。 |
| 20 | minor | §7.4 平移後重鋪 | 採納：重新進入 while 迴圈、不再平移、後置斷言。 |
| 21 | blocker | §10.4 鎖步結構 | 採納（D33）：事件驅動分支、$O(nBK)$、`legacy_mode` 鎖步；§14 與測試更新。 |
| 22 | blocker | §4.4／§8.2／§10.2 方向交替 | 採納（D34）：轉移 4 型 $(n,n,4)$、Eq.(25) 放寬、$T_{proxy}$ 單向胞處理、契約與測試更新。 |
| 23 | major | §10.5 能量修復 | 採納：自 $x_j$ 所在條帶截斷＋暖啟動 IDP；靜態濾網 `static_ok`。 |
| 24 | major | §5.5 $e^{alt}$ | 採納：EnergyBudgetTable 以 AVOID＋ENVELOPE 建真實表，`point_wh` 上界化含爬升項。 |
| 25 | major | §8.7 落點 | 採納（D40）：先 $H$、備降需警告、$C_i$ 以 $H$ 計、多架次首架次落 $H$。 |
| 26 | major | §5.1 $P_c^{ref}$ | 採納（D39）：`reference_battery_wh`、測試 `pack_size_invariant_power`。 |
| 27 | major | §10.3 剪枝 → FAIL | 採納：逐機 RTB、其餘機續行（與 D33 合併實作）。 |
| 28 | major | §12.1 受影響集合 | 採納：定義 A、凍結語意改為尾端追加。 |
| 29 | major | §12.9 氣象中止 | 採納：空集合退為最小超限站點＋警告、依能量餘裕排序、等待前檢查能量。 |
| 30 | major | §8.4 轉移能量 | 採納：$e^{trans}$ 含爬升／下降功率、`entry_e` 同式、§10.1 保守初值。 |
| 31 | major | §9.3 循環／分母／下限 | 採納：$S$ 以 $\lambda=\min$ 濾網、分母 $\le0$ → `energy_overhead`、部分覆蓋。 |
| 32 | major | §9.5 $k_A$ 枚舉 | 採納：候選對併入拍賣、Phase 3 局部微調。 |
| 33 | minor | §9.2 $T^{oper}$ 雙計 | 採納：明文區分，$\lambda$ 下限與 $\tilde T$ 用 `T_single_s`。 |
| 34 | minor | §5.5 檢查止於 $P_{tc}$ | 採納，missed approach 另檢。 |
| 35 | minor | §5.6 vs §12.4 | 採納：未驗證用 $E_{pred}$、驗證後限幅 ±10%。 |
| 36 | minor | §10.4 beam 排序 | 採納：以 $W$ 排序。 |
| 37 | minor | 契約索引約定 | 採納：$idx=(dir+1)/2$、以條帶自身掃向為語意（§8.2、§8.9、契約 §8）。 |
| 38 | minor | §5.3 $e^{LDG}$ 10 m 規則 | 採納：$t_{dn}=(H_{ts}-10)/v_{z,dn}+10/0.5$，共用函式；A 81.2 Wh（含減速段能量；純懸停修正 79.7）。 |
| 39 | minor | §9.4 Phase 2/3/4 | 採納：`no_feasible_insertion`、MOVE/SWAP/BALANCE 呼叫 `hard_feasible` 與 $\lambda$ 下限。 |
| 40 | minor | H3 提早到達 | 採納：延後起飛／等待盤旋、完成時刻驗證。 |
| 41 | minor | k-nearest 兩端狀態 | 採納：（1）改為「唯一狀態 × 4 型中相容 2 型」，同向轉場使兩端皆合法；（2）胞完成偏好。 |
| 42 | minor | H2 迴轉帶 | 採納：$Q\oplus d_{turn}$、裁切先內縮、失敗改方向。 |
| 43 | blocker | 契約 §W.7 makespan | 採納重算，但**結論修正**：審查值 1608 s 誤用左端進入之作業時間（735.1 s）；右端進入序列 1W,4E,2W,5E,3W 的作業時間為 806.9 s，兩側總時間僅差 0.3 s（本版 1696.7 vs 1697.0 s）。§W.7 明寫「兩側擇優、幾乎持平、測試不斷言進入側」。配合其他修正（$h_{op,A}=160$、$d_{tr}=468/384$、加減速段、10 m 規則）$C_{A1}=28.3$ min、makespan_final 29.3 min。 |
| 44 | blocker | 契約 §W.4／B1 終端側 | 採納：全表以正確終端側重算（§W.4）；B1 最佳序列 1E,3W,2E（$n_B=3$ 於 $h_{op,A}=160$）23.5 min；$k_A=5$ 與 $k=6$ 精算後差 <1 s → 測試 `k_A in {5,6}`。 |
| 45 | blocker | 契約 §0 依賴環 | 採納：新增 `context.py`、`routing.py`、`preflight.py`，更新依賴表。 |
| 46 | major | 契約 CostCache 索引 | 採納（同 #37），`T_transfer[l,m,k]=tr+op[m,idx_m]`。 |
| 47 | major | 契約高度基準 | 採納（D42）。 |
| 48 | major | 契約 stage_cost 向量化 | 採納：`IDPNode`、候選矩陣簽章；事件驅動後節點數 $\approx1.5\times10^3$，效能表改寫；`runtime_under_1s` 排除初始化。 |
| 49 | major | 契約 PlannerConfig 缺欄位 | 採納：新增全部欄位；`f_dep` 依 #82 放入 PlatformSpec；`legacy_mode` 明列改寫欄位。 |
| 50 | major | 契約 PayloadRequirement | 採納（D43）。 |
| 51 | major | 契約 §W 座標符號 | 採納：保留右法向定義，§W 座標改 $y=-v$、`sweep_origin`、word 改 RSR／LRL、測試只斷言長度。 |
| 52 | major | 契約 `altitude_band` 循環 | 採納：胞多邊形層級定義（§4.6）。 |
| 53 | major | 契約 `plan_mission` 出口狀態 | 採納：$S_i=S_{acc}$ 或 $S_{exit}$，走廊段不含 AVOID，`takeoff_corridor` 用 `cache.entry` 路徑。 |
| 54 | major | 契約梯形公式 | 採納：§4.3 明寫除以 $\sin$ 形式。 |
| 55 | major | 契約 `residual_k_enumeration` | 採納：每構型索引、`add_strips/remove_strips`、`refresh_entry` 完整簽章、§7.5 只在 $v$ 方向擴。 |
| 56 | minor | 契約 `e_landing_wh` | 採納（同 #38）：降落 110/100 s。 |
| 57 | minor | 契約 §W.7 下降距離 | 採納：883／1060 m。 |
| 58 | minor | B 盤旋數值格式 | 採納：39.7 s／35.8 m、41.9 s／37.8 m，加不變量。 |
| 59 | minor | CorridorSchedule 區間 | 採納：$[t_{arr}(P_{tc})-30,\ t_{land}+90]$、整數圈取整。 |
| 60 | minor | `models` 鍵 | 採納：`models[uav_id]`、`models_rep[cls]`。 |
| 61 | minor | `exact_time` 定義 | 採納：固定 skip-$k$ 序列、兩側平均。 |
| 62 | minor | discretize／Phase／TargetCue／ground_speed／strip_progress | 採納。 |
| 63 | minor | PlatformSpec 與 wind 重複、§W.6 語法 | 採納：wind 版為薄包裝；關鍵字呼叫。 |
| 64 | major | payload_kg | 採納：`SortieEnergy.payload_kg`、推估與警告、PRECHECK 6、測試。 |
| 65 | major | §6.1 PayloadRequirement 重複 | 採納（同 #50）。 |
| 66 | major | 基準分鐘模式／nominal | 採納：`pack_nominal_wh`、解析順序、NO_GO、示例值警告。 |
| 67 | major | c2_link 逐構型／terrain_clearance 優先序 | 採納。 |
| 68 | major | 感測器示例值旗標 | 採納：`is_default_example`、警告、§16.2 標示、§W 註明。 |
| 69 | major | UAVInstance 多感測器 | 採納：`sensors` tuple、`available_kinds`、both 由 A 獨立完成之寬度規則。 |
| 70 | minor | Beaufort 陣風警告 | 採納：`BEAUFORT_UPPER_MPS` 表、`gust > upper`；§W.2 補結果。 |
| 71 | minor | HERO 覆寫／$d_{tr}$ 公式 | 採納：`transition_cmd()` 用公式（468/384）、`transition_ground_run_m` 為可選覆寫；§W 數值同步。 |
| 72 | minor | provisional_supplementary | 採納。 |
| 73 | minor | platform_overrides | 採納。 |
| 74 | minor | $R_{eff}$ 風層 | 採納：$W_{R,c}$ 與 `r_eff_wind_policy`。 |
| 75 | minor | Site AGL／WindLayer z_ref／GeoJSON | 採納。 |
| 76 | minor | max_agl 預設 | 採納 warning 路徑（非 NO_GO），記錄來源。 |
| 77 | minor | usable_range_fraction 自洽 | 採納：建構期 assert 與測試。 |
| 78 | minor | 無懸停不變量 | 採納：Waypoint 不變量、`check_kinematics`、測試。 |
| 79 | minor | FW_ACCEL | 採納：seg2b（含 FW_DECEL 對稱段）、$S_{acc}$；§W.6 更新。 |
| 80 | minor | 硬編碼常數 | 採納：全部進 PlannerConfig；§17.1(3) 引用 spec 角度。 |
| 81 | minor | battery_temp_c | 採納。 |
| 82 | minor | $f_{dep}$ provisional／偏差警告 | 採納：`reference_test_depth_of_discharge`、`endurance_range_inconsistent`。 |

**未採納或部分採納之說明**：(#5) 未採用「垂直段改由 $H_{tc}$ 下降」——表 1 已定義轉換起始高度 $H_{ts}$，改為自 $H_{tc}$ 懸停下降會使懸停時間增加 40 m/1 m/s=40 s、能量 +27 Wh（A），且與表 1 語意不合；改以旋翼輔助轉換段解釋既有幾何，並提供 `fw_slope` 選項。(#43) 審查給出的 1608 s 與「右端進入較優」結論經重算不成立（見上表），但其指出的「IDP 必須在兩側擇優」與「測試不得綁定固定進入側」完全採納。(#76) 採 warning 而非 NO_GO：120 m AGL 為多數法域的通用預設，缺填時仍應可規劃並明示來源。
