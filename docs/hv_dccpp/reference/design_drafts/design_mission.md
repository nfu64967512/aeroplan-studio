# HARVEST 演算法設計（任務／能量／配置優先版）

Heterogeneous-VTOL Dynamic Complete Coverage Path Planning：Pioneer PNP（構型 A）+ HERO PNP（構型 B）兩構型 lift-and-cruise 4+1 VTOL 群飛偵查路徑演算法。
基礎：Wang et al. 2025, *Defence Technology* 47:197–215（下稱「論文」）。本文件所有平台數值取自 `platform_params.md` 表 1，任務輸入取自表 2，未更改任何數值；「待填入／暫定」項一律以「可設定參數 + 預設值 + 標示待實測」處理。

---

## 1 問題定義與假設

### 1.1 問題敘述

給定：
- 兩構型 VTOL 機隊 $U = U^A \cup U^B$，$n_U = n_U^A + n_U^B$；每架 $U_i$ 有構型 $c(i)\in\{A,B\}$、當架次 AUW $m_i$、可用電量 $E^{avail}_i$（Wh）。
- 任務區域集合 $Z=\{Z_k\}$，每區附帶：多邊形（GeoJSON）、感測需求 $\sigma_k\in\{\text{lidar},\text{rgb},\text{both},\text{any}\}$、優先度 $val_k$、時間窗 $[t^{open}_k,t^{close}_k]$、酬載規格需求（GSD、重疊率、點密度）。
- 環境：DEM/DSM、障礙物（點／線／面 + 高度）、高度分層風場 $\mathcal W(z)=(W(z),\theta_w(z))$、C2 鏈路覆蓋（半徑或多邊形）、空域（高度上限、禁區、時間窗）、氣象門檻。
- 起降點 $H_0$（base）與備降點集合 $\mathcal A=\{a_1,\dots\}$（皆含 WGS84 + MSL/AGL）。

求：每架機的完整三維航跡 = 起飛走廊 → 進入路徑 → {作業路徑、轉移路徑}* → （可選：跨區進入）* → 返航路徑 → 降落走廊，使得

$$\min\ C_{\max} = \max_{i\in U}\ C_i,\qquad C_i = t^{TO}_i + T^{entry}_i + \sum T^{oper}_i + \sum T^{trans}_i + T^{RTB}_i + T^{LDG}_i$$

（min-max 完工時間，**含返航與降落**；次要目標 $\sum_i C_i$ 作 tie-break），受限於：完全覆蓋、感測需求、能量可行（任一航點剩餘能量可達最近備降點且任務結束仍保有 30%）、鏈路／半徑、空域／禁區／時間窗、氣象、運動學（Dubins、坡度）、防撞。

### 1.2 兩層偵查概念（two-tier reconnaissance）

| | 第一層（Pioneer PNP, A） | 第二層（HERO PNP, B） |
|---|---|---|
| 責任區 | 遠距（進入時間長）、大面積、寬條帶可用、LiDAR 需求、需長時間佇留（loiter）之區域 | 近距、小面積／狹窄／凹角胞、RGB/GSD 需求區、目標再確認（cue-and-confirm） |
| 決定依據 | 3.4 節能量可用時間 $T^{use}$ 較長（$0.7\times176$ min 級）、對稱半徑 66–72 km | 轉彎半徑小（120 m）→ 窄胞轉彎浪費少；可用半徑 39–43 km |
| 資料流 | LiDAR 點雲即時概略檢測 → 產生「目標線索」(cue) | 接收 cue → 插入近距 RGB 確認任務 |

兩層並非固定分工，而是透過 6 節的「構型適配度」$S_k^c$ 與硬限制自動導出；當某構型缺席時，另一構型在感測需求允許下接手。

### 1.3 假設

1. 兩構型皆為複合翼：懸停只出現在垂直起降、轉換、緊急狀態；**任務中一切轉彎為固定翼 Dubins（半徑 $R^c=$ `turn_radius_planning_m`）**，不存在懸停定點轉向。
2. 巡航以固定空速 $V_a^c=$ `cruise_airspeed_mps` 飛行；地速由風三角決定（3.3）。作業段可因感測器速度上限降速，但不低於 `min_mission_airspeed_mps`。
3. 風場在規劃週期內視為準靜態（分層、可分區）；更新由 10.5 節事件處理。
4. 一次規劃週期內障礙物與 DEM 為靜態。
5. 每架機一次架次（sortie）以基地起飛、基地或備降點降落；多架次為選項（6.6）。
6. 感測器僅在作業路徑上開機（與論文相同）；進入／轉移／返航段不計覆蓋。
7. 高度：條帶寬度以該條帶上「最小 AGL」計算（保守，見 3.2 與 9 節）。

---

## 2 符號表

| 符號 | 意義 |
|---|---|
| $c\in\{A,B\}$ | 構型；$A$=Pioneer PNP，$B$=HERO PNP |
| $U^c$, $n_U^c$ | 構型 $c$ 的機集合與數量 |
| $m_i$, $m^{ref}_c$ | 當架次 AUW、基準 AUW（24.6 / 6.65 kg） |
| $V_a^c$, $V_g$ | 巡航空速、地速 |
| $R^c$ | 規劃轉彎半徑（200 / 120 m） |
| $\gamma^{\uparrow}_c,\gamma^{\downarrow}_c$ | 規劃爬升／下降角（3° / 4°） |
| $h^{ts}_c,h^{tc}_c,h^{min}_c$ | 轉換起始 AGL、轉換完成 AGL、最低固定翼作業 AGL |
| $E^{nom}_c$ | 標稱電池能量（Wh，待填入） |
| $E^{avail}_i$, $E^{use}_i$ | 當架次可用電量、扣除保留與保守係數後可規劃能量 |
| $P_c(m)$, $P_h(m)$, $P_{tr}(m)$ | 巡航、懸停、轉換功率（AUW 相關） |
| $r=0.30$ | `reserve_fraction` |
| $f_u\in[0.55,0.60]$ | `usable_range_fraction` |
| $D^{use}_c$ | `usable_distance_km`（133–145 / 79–86） |
| $\rho^{sym}_c$ | `symmetric_mission_radius_km`（66–72 / 39–43） |
| $Z_k$, $\mathcal C_k=\{Q_{k,q}\}$ | 任務區域、其凸胞（cell） |
| $\sigma_k$ | 感測需求 {lidar, rgb, both, any} |
| $w^c(h)$, $s^c$ | 構型 $c$ 在 AGL $h$ 的條帶寬（swath）、條帶間距 |
| $L_{q,l}$, $n_{q}$ | 胞 $q$ 第 $l$ 條作業路徑、路徑數 |
| $p^L_{q,l},p^R_{q,l}$ | 作業路徑左右端點 |
| $\vec t^{\,oper}_{i,q,l}=[t^{LR},t^{RL}]$ | 二元作業時間 |
| $\vec t^{\,in}_{i,q,l}$, $\vec t^{\,trans}_{i,q,l,l'}$ | 二元進入／轉移時間 |
| $T_{i,q}$, $E_{i,q}$ | 路徑時間與能量矩陣（對應論文 $D_{i,k}$） |
| $E_{i,q}$ (決策) | 決策矩陣 $\in\{-1,0,1\}$（同論文 Eq.14；能量矩陣以 $\mathcal E$ 表示以免混淆） |
| $J^c_q$, $S^c_q$, $\lambda^c_q$ | 配置指標、構型適配度、指派架數 |
| $C_i$, $C_{\max}$ | 機 $i$ 完工時間（含返航）、makespan |
| $t^{RTB}(x)$, $e^{RTB}(x)$ | 由狀態 $x$ 返航至基地的時間／能量 |
| $e^{alt}(x)$ | 由狀態 $x$ 到最近備降點的能量 |
| $\mathcal P_{link}$, $\rho^{link}$ | 鏈路覆蓋多邊形、鏈路半徑 |
| $\beta$ | IDP beam width |

---

## 3 平台／感測器／能量模型

### 3.1 平台運動學（承接論文 Eq.1，加入 VTOL 模式）

固定翼段沿用論文 Eq.(1)，但 $V_i$ 改為地速向量（風三角），$R_{min}\to R^c$，$\gamma_i\in[-\gamma^{\downarrow}_c,\gamma^{\uparrow}_c]$。狀態機：

```
HOVER_CLIMB (v_z = v_z_up, P_h)  →  TRANSITION_OUT (h^ts→h^tc, 逆風, P_tr, t_tr_out)
  →  FW_CLIMB (γ ≤ 3°, 需要時 orbit 半徑 ≥ R^c)  →  FW_CRUISE / FW_OPER
  →  FW_DESCENT (γ ≤ 4°)  →  TRANSITION_IN (P_tr, t_tr_in)  →  HOVER_DESCENT (v_z_dn, P_h)
```

風中轉彎的地面軌跡半徑放大（用於轉移／進入路徑的可行性與長度上界）：
$$R^c_{g} = R^c\cdot\left(1+\frac{W}{V_a^c}\right)^2 \quad(\text{保守上界})$$
Dubins 幾何仍以 $R^c$ 生成（空速座標系），轉換到地面座標時以 $R^c_g$ 檢查障礙淨空。

爬升所需水平距離（推論 2）：$d^{\uparrow}(\Delta h)=\Delta h/\tan\gamma^{\uparrow}_c\approx 19.1\,\Delta h$；下降 $d^{\downarrow}(\Delta h)=\Delta h/\tan\gamma^{\downarrow}_c\approx 14.3\,\Delta h$。爬升率上限 `max_fw_climb_rate_mps`（1.13 / 1.00）、下降率上限（1.50 / 1.33）與空速一致，作為交叉檢查。

### 3.2 感測器模型（取代論文 Eq.2–3 的前視梯形；前視梯形保留為選項 `fov_model=trapezoid`）

**LiDAR（構型 A 基準）**
$$w^{A}(h)=2h\tan\frac{\phi_{use}}{2},\qquad s^A = w^A(h_{\min})\,(1-o_{side})$$
$$V_g^{oper,A}\le \frac{PRR\cdot\eta_{ret}}{\rho_{req}\cdot w^A(h)}$$
$\phi_{use}$ = 可用掃描角（`lidar_fov_usable_deg`，預設 70°），$PRR$ 脈衝率，$\eta_{ret}$ 有效回波比，$\rho_{req}$ 需求點密度（pts/m²）。

**相機（構型 B 基準）**
$$w^{B}(h)=h\frac{S_w}{f},\quad \ell^{B}(h)=h\frac{S_h}{f},\quad GSD(h)=h\frac{p_{pix}}{f}$$
$$s^B=w^B(h_{\min})(1-o_{side}),\qquad V_g^{oper,B}\le \ell^B(h)\,(1-o_{fwd})\,f_{shutter}$$
GSD 需求給出高度上限 $h\le GSD_{req}\cdot f/p_{pix}$；作業高度 $h^{oper}_q=\text{clip}(h_{GSD},\,h^{min}_c,\,h^{ceil})$。

**共同**：條帶寬以條帶上最小 AGL $h_{\min}$ 計算（陡地形保證覆蓋）。若 $V_g^{oper}$ 上限 $<$ `min_mission_airspeed_mps`（HERO 取 15.0）→ 提高高度或降低重疊率，否則該區對此構型不可行。

感測需求相容性 $\kappa(\sigma_k, c)$：

| $\sigma_k$ | A（LiDAR，可選配 RGB 快視相機 `A_has_rgb`） | B（RGB） |
|---|---|---|
| lidar | 1 | 0 |
| rgb | `A_has_rgb`? 1 : 0 | 1 |
| both | `A_has_rgb`? 1 : 0（否則需 A、B 各飛一次） | 0（僅能承擔 rgb 子任務） |
| any | 1 | 1 |

`both` 且 A 無 RGB 時：區域複製為兩個子任務 $Z_k^{lidar}, Z_k^{rgb}$，各自配置。

### 3.3 風三角與地速

航向（course）$\chi$，風向「吹向」角 $\theta_w'$（氣象風向 + 180°），$\Delta=\chi-\theta_w'$：
$$V_g(\chi,z)=\sqrt{(V_a^c)^2-\big(W(z)\sin\Delta\big)^2}+W(z)\cos\Delta,\qquad \text{需 } V_a^c>W|\sin\Delta|$$
航段時間 $t=\ell/V_g$（$\ell$ 為地面距離）。作業路徑兩方向時間不同（$t^{LR}\ne t^{RL}$）→ 論文「二元」成本的自然來源。Dubins 弧段：以弧上離散點（論文 Eq.28）逐段用該點航向計算 $V_g$ 積分。

### 3.4 能量模型（由表 1 建立）

**巡航功率**（由基準續航推得，AUW 以 $(m/m^{ref})^{1.5}$ 修正）
$$P_c^{ref}=\frac{f_{dep}\,E^{nom}_c}{T^{ref}_c},\qquad P_c(m)=P_c^{ref}\Big(\frac{m}{m^{ref}_c}\Big)^{1.5}$$
$T^{ref}_c$=`reference_endurance_min`（176 / 136 min），$f_{dep}$=基準測試耗用比例（預設 1.0）。$E^{nom}_c$ 待填入；未知時以「基準巡航秒」為能量單位：$1\,\text{unit}=P_c^{ref}\cdot 1\,\text{s}$，$E^{avail}$ 以 SoC 比例輸入。

**交叉檢查（表 1 一致性）**：$R^{ref}_c/T^{ref}_c$ = 242 km/176 min = 22.9 m/s（A）、144/136 = 17.6 m/s（B）；與 $V_a^c$（21.5 / 19.0）差 +6% / −7%。取保守：能量用 $T^{ref}$，距離用 $\min(V_a^c T^{ref}_c, R^{ref}_c)$ = 227 km（A）、144 km（B）。

**懸停功率**（動量理論，或比例法）
$$P_h(m)=\frac{(mg)^{1.5}}{FM\sqrt{2\rho_{air}A_{disk}}}\quad\text{或}\quad P_h(m)=k_h P_c^{ref}\Big(\frac{m}{m^{ref}_c}\Big)^{1.5},\ k_h=\texttt{hover\_power\_ratio}\ (\text{預設 } 5.0)$$

**轉換功率** $P_{tr}=P_h+P_c$（雙系統同時運轉），時長 $t^{out}_{tr}$（預設 20 s）、$t^{in}_{tr}$（25 s）。

**爬升功率** $P_{\uparrow}(m)=P_c(m)+mg\,v_z/\eta_p$（$\eta_p$ 推進效率 0.6）；下降 $P_{\downarrow}=\max(P_c(m)\,\eta_{idle},\,P_c(m)-mg v_z/\eta_p)$，$\eta_{idle}=0.3$。

**每段能量**：
$$e_{seg}=P_{mode}(m)\cdot t_{seg}\cdot k_{unc}$$
$k_{unc}$ 不確定性乘數：`battery_monitor_validated=false` → $k_{bm}=1.15$；`airspeed_sensor_validated=false` → $k_{as}=1.05$（地速估計誤差）；$k_{unc}=k_{bm}k_{as}$。溫度 $T<10^\circ$C 時容量係數 $\eta_T=1-0.01(10-T)$。

**起降固定成本**（每架次）：
$$e^{TO}=P_h\frac{h^{ts}_c}{v_{z,up}}+P_{tr}t^{out}_{tr},\qquad e^{LDG}=P_{tr}t^{in}_{tr}+P_h\frac{h^{ts}_c}{v_{z,dn}}+P_h t_{hover,res}$$
（$t_{hover,res}$ 懸停對位裕度 15 s。）

**可規劃能量**
$$E^{use}_i=\eta_T\,E^{avail}_i-r\,E^{nom}_c-e^{TO}-e^{LDG}$$
（30% 保留以標稱容量計，任務結束後仍需持有。）

**可用距離交叉檢查**：計畫的巡航等效距離 $D^{plan}_i=\sum_{seg}\dfrac{e_{seg}}{P_c(m_i)}V_a^c$ 必須 $\le D^{use}_c$（上限 145 / 86 km，`usable_distance_km` 上緣；`battery_monitor_validated=false` 時取下緣 133 / 79）。從 $f_u/(1-r)=0.786\text{–}0.857$ 可回推表 1 隱含的「任務損耗係數」$\eta_m\approx0.8$；本模型已明確計入轉彎、爬升、起降，仍保留 $D^{use}$ 為獨立上限。任一航點距基地投影距離 $\le\rho^{sym}_c$（對稱半徑）作第二交叉檢查。

### 3.5 能量安全計畫（energy-safe plan）

計畫在每個航點 $x_j$（含 Dubins 離散點）必須滿足：
$$E^{rem}_i(x_j)\ \ge\ r\,E^{nom}_c+\min_{a\in\mathcal A\cup\{H_0\}} e^{alt}(x_j,a)$$
$$e^{alt}(x,a)=k_{unc}\Big[P_c\frac{d_{Dubins}(x,a)}{V_g^{worst}}+P_{\downarrow}t_{\downarrow}+e^{LDG}\Big],\quad V_g^{worst}=V_a^c-W_{\max}(z)$$
備降點必須位於鏈路多邊形內（`alternate_landing_sites_required=true` 為硬性：無備降點 → 不放飛）。快速估計用直線距離；最終驗證用 Dubins + 障礙繞行。

```
function ENERGY_SAFE_CHECK(route_i, E_use_i):
    E_rem = E_use_i + r*E_nom + e_LDG    # 起飛後可用（保留與降落尚未扣）
    for seg in route_i.segments:          # 含 RTB 段
        E_rem -= e(seg)
        for x in seg.discretised_points():
            e_alt = min over a in ALT_SITES ∪ {H0} of e_alt(x, a)
            if E_rem - e(seg remainder to x) < r*E_nom + e_alt: return INFEASIBLE(x)
    return FEASIBLE, E_rem   # E_rem 此時應 ≥ r*E_nom + e_LDG
```

---

## 4 任務輸入與前置檢查

### 4.1 輸入（表 2 全部）

| 輸入 | 內部表示 |
|---|---|
| 起飛點與備降點 | $H_0$, $\mathcal A$：WGS84→ENU；每點 MSL、AGL、`vtol_site_clear_radius_m`（8 / 6）、`obstacle_horizontal_buffer_m`（30 / 25）檢查 |
| 任務區域 | Polygon → 5 節分解；屬性 $\sigma_k, val_k$, 時間窗 |
| 地形模型 | DEM/DSM 網格 + 雙線性內插（論文 Eq.4） |
| 障礙物 | 點／線／面 + 高度 → keep-out 多邊形（水平緩衝 $b_h$）+ 垂直淨空 $b_v$ |
| 風場 | 分層表 $\{(z_j, W_j, \theta_j)\}$，可分區 |
| 當架次 AUW | $m_i$；檢查 $m_i\le$ `mtow_kg`（24.9 / 7.0），酬載 $\le$ `payload_max_kg`（5.0 / <1.0） |
| 可用電量 | $E^{avail}_i$ Wh 或 mAh×V 或 SoC |
| 酬載任務需求 | LiDAR $\phi_{use}$, PRR, $\rho_{req}$ ／ 相機 $f, S_w, S_h, p_{pix}$, $GSD_{req}$, $o_{side}, o_{fwd}$, $f_{shutter}$ |
| C2 鏈路覆蓋 | $\rho^{link}$ 或 $\mathcal P_{link}$；`c2_link_range_km` 待填入 → 預設保守 15 km，標示待實測 |
| 空域與法規 | $h^{ceil}$（預設 120 m AGL）、禁區多邊形、時間窗 |
| 作業氣象門檻 | $W_{go}, G_{go}, vis_{go}, rain_{go}, T_{range}$ 與中止值 |

### 4.2 前置檢查（Pre-flight gate）

```
function PREFLIGHT_GATE(mission, fleet, wx):
    # (1) 氣象 go/no-go
    assert wx.mean_wind ≤ W_go^c  for every c in fleet   # 預設 min(8.0 m/s, 0.45·V_a^c) → A 8.0, B 8.0(0.45·19=8.55)
    assert wx.gust ≤ G_go (預設 12 m/s); wx.visibility ≥ 3 km; wx.precip == none
    assert T_min ≤ wx.temp ≤ T_max (預設 −10..40 °C)
    # Beaufort 5 (8.0–10.7 m/s) 為平台宣稱，非放行門檻 → W_go 取其下緣以下
    # (2) 起降點
    for site in {H0} ∪ ALT: assert clear_radius(site) ≥ vtol_site_clear_radius_m^c
                              assert dist(site, obstacles taller than site+h_ts) ≥ obstacle_horizontal_buffer_m^c
                              assert site ∈ P_link
    assert alternate_landing_sites_required → |ALT| ≥ 1
    # (3) 機體
    for i: assert m_i ≤ mtow; payload_i ≤ payload_max; E_use_i > 0
    # (4) 鏈路／半徑：任務區域裁切
    for Z_k: Z_k ← Z_k ∩ P_link ∩ Disc(H0, ρ_sym^c_max) \ NoFly ; 記錄裁掉部分為 UNCOVERABLE
    # (5) 空域：h_ceil vs h_min^c ；若 h_ceil < h_min^c → 該構型不可用（A: 120 = 120 恰好可用，僅允許 h = 120 AGL）
    # (6) 感測可行：對每 (k,c) 計算 h_oper, s^c, V_g^oper；若 V_g^oper < min_mission_airspeed → 標記 (k,c) 不可行
    # (7) 轉換裕度：airspeed_sensor_validated=false → 轉換段風限 W ≤ 0.5·(transition_airspeed − stall) 額外檢查
    return GO / NO-GO(reason list)
```

---

## 5 區域分解

（本節為概要；幾何細節由區域分解代理深化。）

1. **簡化**：Douglas–Peucker（容差 $\epsilon_{DP}$ = 0.25·$\min_c s^c$），不整體凸化。
2. **凸分割**：Hertel–Mehlhorn 或最小寬度導向的凸分割 → 凸胞 $\mathcal C_k=\{Q_{k,q}\}$；合併相鄰胞若合併後最小寬度方向一致且轉彎數不增。
3. **掃描方向**：每胞候選方向 = 最小寬度方向 $\alpha_q$（論文）± 各邊方向；評估「總時間（含風）」：
   $$T_q^c(\alpha)=\sum_l \min(t^{LR}_l,t^{RL}_l)\big|_{\text{交錯}} + (n_q-1)\,\bar t^{turn}_c(s^c,\alpha)$$
   取最小者。風強時「垂直於風」的掃描方向常優於最小寬度方向（往返地速對稱）。
4. **條帶**：以構型 $c$ 的 $s^c$（由該胞保守 AGL 計算）產生 $L_{q,l}$，端點依論文 Eq.(8)(9) 並加感測端點延伸 $\delta^{ext}_c$（相機：$\ell^B/2$；LiDAR：0）。每胞為 A、B 各生成一套條帶（A 用 $s^A$，B 用 $s^B$）。
5. **殘帶（residual band）指派**：寬胞以 $w^A$ 覆蓋後殘餘寬度 $w^{res}<s^A$ 且 $\sigma_k\in\{\text{rgb},\text{any}\}$ 時，殘帶可作為獨立窄胞交給 B（僅當 $S^B_{res}>S^A_{res}$）。
6. **胞屬性**：面積 $A_q$、最小寬度 $w_{q,\min}$、周長、到基地距離 $d_q$、地形高差、$n_q^c$、$T_q^c$（單機覆蓋時間）。

---

## 6 任務配置（Allocation）——核心

### 6.1 硬限制（hard constraints）

對胞 $q$、構型 $c$、機 $i$：
- H1 感測：$\kappa(\sigma_k,c)=1$。
- H2 鏈路／半徑：$Q_q\subset\mathcal P_{link}$ 且 $\max_{x\in Q_q}\|x-H_0\|\le\rho^{sym}_c$（保守用下緣 66 / 39 km）。
- H3 空域：$h^{oper}_q\le h^{ceil}$，$Q_q\cap$ NoFly $=\emptyset$，抵達時間 $\in[t^{open},t^{close}]$。
- H4 能量：單機承擔份額 $T_{i,q}/\lambda$ 時 $e^{TO}+e^{in}+e^{oper}+e^{trans}+e^{RTB}+e^{LDG}\le \eta_T E^{avail}_i - rE^{nom}$，且 3.5 節逐點檢查。
- H5 感測速度：$V_g^{oper,c}\ge$ `min_mission_airspeed_mps`。
- H6 幾何：$w_{q,\min}\ge s^c$（否則單條帶）且胞內可容納半徑 $R^c_g$ 之轉彎（否則轉彎在胞外進行——允許，但需檢查禁區）。

### 6.2 構型適配度 $S^c_q$（configuration suitability，0–1）

$$S^c_q=\kappa(\sigma_k,c)\cdot\prod_{H\in\{H2..H6\}}\mathbb 1_H\cdot\Big[\nu_1 s^{width}+\nu_2 s^{area}+\nu_3 s^{dist}+\nu_4 s^{turn}+\nu_5 s^{endur}+\nu_6 s^{gsd}\Big]$$

- $s^{width}=1-\dfrac{\text{frac}(w_{q,\min}/s^c)\cdot s^c}{w_{q,\min}}$（條帶整除度；殘帶少者高）。
- $s^{area}$：A 偏好大面積：$s^{area}_A=\text{sat}(A_q/A^*)$，$s^{area}_B=1-s^{area}_A$；$A^*$=A 單條帶長 5 km × 寬 $s^A$ 的量級（可設）。
- $s^{dist}$：A 偏遠：$s^{dist}_A=\text{sat}(d_q/\rho^{sym}_B)$，$s^{dist}_B=1-\text{sat}(d_q/\rho^{sym}_B)$；$d_q>\rho^{sym}_B$ 時 B 已被 H2 排除。
- $s^{turn}=1-\dfrac{(n^c_q-1)\bar t^{turn}_c}{T^c_q}$（轉彎時間佔比低者高；窄胞下 B 的 $R^B=120$ 明顯占優）。
- $s^{endur}=1-\dfrac{T^c_q}{T^{use}_c}$（單機需時對可用時間比；A 的 $T^{use}$ 大）。
- $s^{gsd}$：rgb 需求時 B 為 1；A（若有 RGB 快視）依 GSD 達成率。

預設 $\nu=(0.15,0.15,0.20,0.20,0.20,0.10)$。

### 6.3 配置指標 $J^c_q$（擴充論文 Eq.30，成本改為時間）

$$J^c_q=\omega_1\frac{\sum_{i\in U^c}\min_l t^{in}_{i,q,l}}{\sum_{q'}\sum_{i\in U^c}\min_l t^{in}_{i,q',l}}+\omega_2\frac{n^c_q}{\sum_{q'}n^c_{q'}}+\omega_3\frac{T^{oper,c}_q}{\sum_{q'}T^{oper,c}_{q'}}+\omega_4\frac{val_q}{\sum_{q'}val_{q'}}+\omega_5\frac{T^{RTB,c}_q}{\sum_{q'}T^{RTB,c}_{q'}}+\omega_6\frac{S^c_q}{\sum_{q'}S^c_{q'}}$$

新增：$\omega_5$ 返航時間（遠區完工晚 → 需更多機）、$\omega_6$ 適配度（適配高的構型多投入）。預設 $\omega=(0.2,0.15,0.2,0.15,0.1,0.2)$；令 $\omega_5=\omega_6=0$、其餘 0.25 即論文。$T^{oper,c}_q=\sum_l\max(t^{LR}_l+t^{RL}_{l+1},\,t^{RL}_l+t^{LR}_{l+1})$ 的時間版（論文 Eq.30 註解）。

### 6.4 構型指派與架數（每構型獨立版 Eq.31）

```
function ASSIGN_CONFIG_AND_COUNT(cells, U^A, U^B):
    for q in cells:
        feasible_c = {c : S^c_q > 0}
        if feasible_c == ∅: mark q UNCOVERABLE; continue
        # 1) 構型偏好：時間效率 × 適配度
        for c in feasible_c: score_c = S^c_q / T^c_q_norm    # T_norm = (t_in + T_oper + T_turn + t_RTB)/T_use^c
        c*_q = argmax score_c ; c2_q = second (備援)
    for c in {A,B}:
        Q_c = {q : c*_q == c}
        for q in Q_c: compute J^c_q over Q_c (6.3)
        for q in Q_c: λ^c_q = clip( round(n_U^c · J^c_q / Σ_{q'∈Q_c} J^c_{q'}), 1, min(n_U^c, n^c_q) )
        # 能量下限：單機份額必須可行
        for q in Q_c: λ^c_q = max(λ^c_q, ceil(T^c_q / (κ_load · T^{use,c} − t_in − t_RTB)))   # κ_load 預設 0.85
        if Σ λ^c_q > n_U^c: 依 J 由小到大遞減 λ（不低於能量下限）；仍超出 → 啟用多架次（6.6）或將 J 最小的胞改派 c2_q
    return {c*_q, λ^c_q}
```

### 6.5 貪婪／拍賣配置（makespan 目標）

**Phase 1 貪婪（論文 Algorithm 1 的時間版）**：胞依 $J^c_q$ 降序，選 $\lambda^c_q$ 架「進入時間最短且能量可行」的 $U^c$ 未指派機。

**Phase 2 拍賣（auction）處理小胞串接**：每架機可承擔多個胞（論文一機一區的放寬）。物品 = 尚未指派或 $\lambda=1$ 的胞；出價 = 邊際完工時間增量。

```
function AUCTION_ALLOCATE(cells, fleet, plan):
    # plan[i] = 有序胞序列 + 目前末端狀態 x_i + 累積時間 C_i(含暫估 RTB) + 剩餘能量
    open = cells not yet assigned (排除 UNCOVERABLE)
    while open ≠ ∅:
        best = None
        for q in open:
            for i in fleet with κ(σ_q, c(i)) = 1:
                for pos in insertion positions of plan[i]:      # 允許插入中間
                    Δ = MAKESPAN_AFTER_INSERT(plan, i, q, pos) − C_max(plan)
                    if not ENERGY_FEASIBLE(plan[i] with q@pos): continue
                    if not TIME_WINDOW_OK(arrival(q), q): continue
                    bid = (Δ, C_i_after, −S^{c(i)}_q)            # 字典序：makespan 增量 → 該機完工 → 適配度
                    if best is None or bid < best.bid: best = (i,q,pos,bid)
        if best is None: 啟用多架次(6.6) 或標記 UNCOVERABLE; break
        apply best; open.remove(q)
    # Phase 3 局部改善（min-max）
    repeat until no improvement or iter > N_ls:
        i_max = argmax_i C_i
        for q in plan[i_max]: for j ≠ i_max: try MOVE(q → j) / SWAP(q ↔ q' ∈ plan[j])
            accept if C_max decreases (tie: Σ C_i decreases) and energy/link feasible
    return plan
```

`MAKESPAN_AFTER_INSERT` 以快速估計：進入 Dubins 時間 + $T^c_q$（單機）+ 至下一胞的 Dubins 時間 + 更新的 RTB 時間。精確值由 8 節 IDP 在配置後計算；若 IDP 結果與估計偏差 > 10%，回饋一次重跑 Phase 3。

### 6.6 多架次（multi-sortie，選項 `multi_sortie=true`）

同一機返回基地換電（$t_{swap}$ 預設 10 min）後再出。等效為在機隊中加入「虛擬機」$U_i^{(2)}$：起始狀態 = 基地、可用時刻 $t^{avail}=C_i^{(1)}+t_{swap}$、能量 = 滿電（新電池 $E^{avail}$ 由使用者給定，否則 $=E^{nom}$）。大胞分割為架次塊：

```
function SPLIT_FOR_SORTIES(q, c, i):
    budget = κ_load·T^{use,c} − t_in(H0→q) − t_RTB(q→H0)
    chunks = []; cur = []; acc = 0
    for l in strips ordered along n̂_q,min:
        t_l = min(t^{LR}_l, t^{RL}_l) + t̄^turn_c
        if acc + t_l > budget: chunks.append(cur); cur = []; acc = 0
        cur.append(l); acc += t_l
    chunks.append(cur)
    return chunks     # 每塊視為獨立子胞，可被不同機／架次承接
```

makespan 目標下多架次只在「所有機皆滿載且仍有未覆蓋」時啟用，且第二架次的完工時間必然計入 $C_{\max}$。

---

## 7 三類路徑成本（時間與能量版）

沿用論文三類路徑與二元結構（Eq.10–12），把長度改為 (時間, 能量) 二元組。

### 7.1 作業路徑
$$\vec t^{\,oper}_{i,q,l}=\begin{bmatrix}t^{LR}\\ t^{RL}\end{bmatrix}=\begin{bmatrix}\ell_{q,l}/V_g(\chi_{LR},z_l)\\ \ell_{q,l}/V_g(\chi_{RL},z_l)\end{bmatrix},\quad e^{oper}=P_c(m_i)\,t\,k_{unc}$$
作業地速取 $\min(V_g,\,V_g^{oper,c})$（感測器速度上限時降空速，不低於 `min_mission_airspeed_mps`）。

### 7.2 進入路徑
起始狀態 $S_i$ = **轉換完成點**（位置 $H_0$ 上風側、航向逆風 $\chi_0=\theta_w'+\pi$、空速 $V_a^c$、高度 $h^{tc}_c$）。
$$\vec t^{\,in}_{i,q,l}=\begin{bmatrix}t(\mathcal D(S_i\to S^L_{q,l}))\\ t(\mathcal D(S_i\to S^R_{q,l}))\end{bmatrix}$$
Dubins 弧段離散（論文 Eq.28/29）逐點以航向求 $V_g$；含爬升段：若 Dubins 長度 $<d^{\uparrow}(h^{oper}_q-h^{tc}_c)$ 則加入爬升盤旋（9.2），時間加 $t^{orbit}$。障礙繞行採論文 Eq.27 的兩段 Dubins（安全半徑 $=R_T+b_h$）或緩衝多邊形可視圖 + Dubins 接合。

### 7.3 轉移路徑
$$\vec t^{\,trans}_{i,q,l,l'}=\begin{bmatrix}t(\mathcal D(S^L_{q,l}\to S^L_{q,l'}))\\ t(\mathcal D(S^R_{q,l}\to S^R_{q,l'}))\end{bmatrix}$$
（與論文 Eq.12 相同的方向約定：由 $L_{q,l}$ 的終端（左端點側，航向 $\psi^L$）轉至 $L_{q,l'}$ 的同側端點反向進入。）

**快速估計（180° 反向、側移 $s=|l'-l|\,s^c$）**：
$$\tilde L^{turn}_c(s)=\begin{cases}R^c\big(\pi+4\arccos\frac{s+2R^c}{4R^c}\big), & 0\le s\le 2R^c\ (\text{CCC 燈泡轉彎})\\ \pi R^c+(s-2R^c), & s>2R^c\end{cases}$$
數值：A（$R$=200 m，$s^A\approx134$ m）相鄰條帶 $\tilde L=1300$ m；隔 2 條（402 m）$\approx630$ m。B（$R$=120，$s^B\approx103$ m）相鄰 $\approx800$ m；隔 2 條（309 m）$\approx446$ m。→ **交錯（skip-k）掃描順序** $k=\lceil 2R^c/s^c\rceil$ 大幅省時；IDP 允許任意 $l\to l'$ 轉移故可自動選出，beam 版本以此估計作啟發。

### 7.4 返航路徑（新增第四類）
$$t^{RTB}_i(S)=t(\mathcal D(S\to S^{app}_{H_0}))+t^{\downarrow}+t^{in}_{tr}+\frac{h^{ts}_c}{v_{z,dn}}$$
$S^{app}_{H_0}$ = 降落走廊入口（逆風、$h^{tc}_c$）；下降段若 Dubins 長度 $<d^{\downarrow}(h-h^{tc}_c)$ → 加下降盤旋。逆風返航：$V_g$ 以實際風三角計算；能量 $e^{RTB}$ 用 $k_{unc}$。

### 7.5 時間／能量矩陣（對應論文 Eq.13/14）
$$\mathcal T_{i,q}=[\vec t_{i,q,l,l'}]_{(n_q+1)\times n_q},\quad \mathcal E_{i,q}=[\vec e_{i,q,l,l'}],\quad E_{i,q}=[e_{i,q,l,l'}]\in\{-1,0,1\}$$
第 0 列為進入 + 作業，其餘列為轉移 + 作業；二元加法規則與方向約束 Eq.(15)(16)(19)–(26) 逐字沿用，只把 $d$ 換成 $t$。每構型各算一套（同構型內機只差起始狀態 → 第 0 列不同、其餘共用）。

---

## 8 異構 IDP（Heterogeneous Improved Dynamic Programming）

### 8.1 狀態與初始化（擴充論文 Eq.32）

$$\Sigma(1)=\Big\{R_{i}=\emptyset,\ T_i(1)=t^{avail}_i,\ E^{rem}_i(1)=E^{use}_i+rE^{nom}+e^{LDG},\ S_i(1)=S^{tc}_i\ \big|\ i\in U_q\Big\},\quad L^{NAD}(1)=L_q$$
$U_q$ 可含兩構型（例如 A 主掃 + B 掃殘帶時，殘帶已分割為獨立胞，故單一胞內 $U_q$ 通常同構型；跨構型混編在同一胞僅發生在 $\sigma=$any 且兩構型皆被指派時，此時各機用各自 $s^c$ 的條帶——**不允許**，改為先以 $s^{\min}=\min_c s^c$ 產生共用條帶，再讓 A 以 $s^A$ 覆蓋時一次「吃掉」$\lfloor s^A/s^{\min}\rfloor$ 條共用條帶；本版本簡化為：混編胞一律用 $s^{\min}$ 條帶，A 每條作業路徑標記覆蓋相鄰 $\lfloor s^A/s^{\min}\rfloor$ 條。）

### 8.2 階段轉移成本（取代論文 Eq.33）

論文 Eq.33 為 $(轉移+作業)/累積距離$（相對增量，天然平衡負載）。本設計以 makespan 下界為主：
$$W\big(\Sigma(s),\,\pi\big)=\max_{i\in U_q}\Big[T_i(s)+t^{trans}_{i,l_i\to\pi_i}+t^{oper}_{i,\pi_i}+\hat t^{RTB}_i(\pi_i)\Big]+\epsilon\sum_i\big[T_i(s)+t^{trans}+t^{oper}\big]+\mu\sum_i\frac{t^{trans}+t^{oper}}{T_i(s)+\tau_0}$$
- 第一項：含返航估計 $\hat t^{RTB}$（直線逆風下界）的 makespan 下界（A* 式啟發）。
- 第二項 $\epsilon=0.05$：總時間 tie-break。
- 第三項 $\mu$：論文 Eq.33 的相對增量項（$\mu=1$ 且前兩項為 0 時退化為論文）。
方向由二元結構決定：進入後第一條作業方向 = 進入側（Eq.26 類比），相鄰轉移方向相反（Eq.25），不同機不由同條路徑進入（Eq.23）。

狀態方程（論文 Eq.34 的 min-max 版）：
$$dp\big(\Sigma(s+1),L^{NAD}\setminus\pi\big)=\min_{\pi\in\Pi(s)}\Big[W(\Sigma(s),\pi)\Big],\quad \Pi(s)=\text{Perm}(L^{NAD}(s),\lambda_q)$$

### 8.3 能量可行剪枝

排列 $\pi$ 被剪枝若任一 $i$：
$$E^{rem}_i(s)-e^{trans}_{i,l_i\to\pi_i}-e^{oper}_{i,\pi_i}<rE^{nom}_c+\min_a e^{alt}\big(p^{end}_{\pi_i},a\big)$$
（到達新路徑終點時仍能到最近備降點）。若某階段所有排列皆被剪枝 → 該機標記 `MUST_RTB`，從 $U_q$ 移除並把剩餘路徑交回配置層（10.3）。

### 8.4 演算法（擴充論文 Algorithm 2：時間成本、beam、能量、返航）

```
Algorithm H-IDP(q, U_q, T_{i,q}, E_{i,q}, β):
 1  Beam ← {Σ(1)}                                  # β = 1 即論文的單狀態貪婪 DP
 2  s ← 1
 3  while ∃ Σ ∈ Beam with L^NAD(Σ) ≠ ∅:
 4      Cand ← ∅
 5      for Σ in Beam:
 6          n_rem ← |L^NAD(Σ)|
 7          if n_rem ≥ λ_q:
 8              Π ← permutations of λ_q paths from L^NAD(Σ)           # A(n_rem, λ_q) 個
 9          else:
10              Π ← { (subset of n_rem UAVs from U_q) × permutations of L^NAD(Σ) }   # A(λ_q, n_rem)，論文 line 12
11              for i not selected: 該機進入 RTB（T_i += t^RTB_i(S_i)）
12          for π in Π:
13              if violates Eq.(23)/(25)/(26) analogues: continue
14              if ENERGY_PRUNE(Σ, π): continue
15              cost ← W(Σ, π)                                     # 8.2
16              Cand.add( (cost, APPLY(Σ, π)) )
17      if Cand = ∅: return FAIL(MUST_RTB set)                    # 交回配置層
18      Beam ← β lowest-cost distinct states in Cand              # 去重：以 (L^NAD, 各機端點與方向) 為鍵
19      s ← s + 1
20  Σ* ← argmin over Beam of  max_i [T_i + t^RTB_i(S_i)]         # 最終以真實 Dubins 返航時間評估
21  for i in U_q: append RTB segment; run ENERGY_SAFE_CHECK(route_i)
22  return ∪_i {R_i, E_{i,q}, T_i, E^rem_i}
```

`APPLY` 更新每機的 $S_i$（新端點、反向航向）、$T_i$、$E^{rem}_i$、決策序列 $R_i$（論文 Eq.22 結構：$e^{in},e^{oper},e^{trans},\dots$）。

### 8.5 跨胞串接
機 $i$ 依配置層順序 $q_1\to q_2\to\dots$：$q_1$ 的 H-IDP 完成後，其末端狀態 $S_i$ 成為 $q_2$ 的初始狀態（第 0 列改為「跨胞進入」Dubins 時間，含高度變化的爬升／下降盤旋）。多機共享同一胞時，各機初始狀態可不同（來自不同前胞），H-IDP 原生支援（論文亦然）。

### 8.6 同構退化
$U_q$ 同構型、$W=0$、$\beta=1$、$\epsilon=0$、第一項移除、$\mu=1$、$E=\infty$、$t^{RTB}=0$ → 第 8.4 演算法逐行等於論文 Algorithm 2（時間 = 距離/V）。

---

## 9 三維高度與起降走廊

### 9.1 坡度受限包絡（取代論文 Eq.36 的單向下降）
路徑等距內插（$\delta d$，論文 Alg.3 line 2）→ DEM 內插（Eq.4）→ 地球曲率修正（Eq.35）→ 期望高度 $z^0_j=z^{g}_j+h^{oper}_q$（或 $+\max(h^{min}_c,$ `terrain_clearance_m`$)$ 於非作業段）。
```
floor_j = z^g_j + clearance_j              # 淨空 = terrain_clearance_m（待任務定義；預設 30 m）+ 障礙物垂直淨空 b_v
fwd[0]  = z_start;  for j: fwd[j]  = max(floor_j, min(z^0_j, fwd[j-1]  + δd·tan γ↑))     # 爬升率受限
bwd[N]  = z_end;    for j↓: bwd[j] = max(floor_j, min(z^0_j, bwd[j+1] + δd·tan γ↓))     # 反向 = 下降率受限
z_j = max(fwd[j], bwd[j]); assert z_j ≥ floor_j  else 路徑需改道(障礙/陡坡不可跟隨) 
```
再以論文 Eq.37/38 梯度下降平滑（限制平滑後仍 $\ge$ floor 且坡度 $\le$ 限制）、Eq.39 刪點。
條帶寬度用該條帶上 $\min_j(z_j-z^g_j)$（保守 AGL）重算，若小於產生條帶時假設值 → 該胞條帶間距縮小並重新分解（迭代至多 2 次）。
高度上限：$z_j-z^g_j\le h^{ceil}$。

### 9.2 起飛走廊與爬升盤旋
```
TAKEOFF_CORRIDOR(H0, c, wind):
  1 垂直爬升 H0 → h_ts^c (60/50 m AGL)，v_z_up
  2 轉換：航向逆風 χ0 = θ_w'+π，加速至 transition_airspeed (16/14) → cruise；高度 h_ts → h_tc^c（80–100 / 70–90，能量用上緣、幾何用下緣）
  3 固定翼爬升 γ ≤ 3° 至 h_oper：需水平距離 d↑ = (h_oper − h_tc)/tan3°
     若進入路徑 Dubins 長度 < d↑ 或走廊內有障礙：插入爬升盤旋 orbit(半徑 R_orb = max(R^c, R^c_g), 中心在 H0 上風側 2R^c)
        每圈爬升 Δh_orb = 2πR_orb·tan3° （A: 65.8 m/圈；B: 39.5 m/圈），圈數 = ceil(剩餘Δh/Δh_orb)
  4 走廊淨空：半徑 vtol_site_clear_radius_m 內無物；高於 H0 + h_ts 之障礙距離 ≥ obstacle_horizontal_buffer_m；走廊多邊形 = 逆風扇形(±30°, 長 d↑) 檢查障礙
```
降落走廊為其鏡像（4° 下降、下降盤旋 $\Delta h_{orb}^{\downarrow}=2\pi R\tan4^\circ$：A 87.9 m/圈、B 52.7 m/圈；轉換前逆風對齊）。

### 9.3 高度分層
A 與 B 任務高度不同（$h^{oper}$ 由感測需求決定；預設 A 150 m AGL、B 100 m AGL，皆 $\le h^{ceil}$）；轉移／返航段使用構型專屬層 $h^{transit}_c$（A: $h^{oper}_A+50$，B: $h^{oper}_B$），$h^{ceil}=120$ m 時退化為時間分離（11 節）。

---

## 10 動態重規劃（Dynamic re-planning）

通用框架（論文 4.3 的擴充）：
```
function REPLAN(event, world, plans, t_now):
    snapshot ← 各機當前狀態 S_i(t_now)、E^rem_i、正在執行之作業路徑 l_i^cur（若 FOV 有效，先完成該條，論文 4.3.2）
    L_uncov ← 全部未覆蓋作業路徑（含中斷者之殘段，重新以中斷點為端點切割）
    Z' ← 依 event 修改胞集合／機隊／風場／限制
    重新計算受影響胞的 T,E 矩陣（只重算第 0 列與受風影響項；快取其餘）
    plan' ← ASSIGN_CONFIG_AND_COUNT → AUCTION_ALLOCATE（以 snapshot 為初始，t^avail_i = 完成當前路徑時刻）→ H-IDP
    驗證 ENERGY_SAFE_CHECK、鏈路、防撞；上傳增量航點（只替換 l_i^cur 之後的段）
```
重規劃時限：$\le$ 2 s（IDP 計算量見 12 節）；超時則先下發「續飛當前路徑 + 保守 RTB」佔位計畫。

### 10.1 UAV 退出（loss / abort）
```
on UAV_LOSS(i):
    fleet.remove(i); L_uncov += paths of i not completed (含當前路徑自中斷點起的殘段)
    if c(i)=A and remaining A = ∅: 對 σ ∈ {lidar, both} 之未覆蓋路徑 → 標記 UNCOVERABLE 並回報；σ ∈ {rgb, any} → 允許 B 接手（用 B 的條帶重新分解該殘區）
    REPLAN(...)   # 論文 4.3.1：同型機接手殘路徑；本設計加入跨構型接手與能量檢查
```

### 10.2 新區域（new area）
```
on NEW_AREA(Z_new, σ, val, window):
    分解 Z_new → cells；PREFLIGHT_GATE 的 (4)(5)(6) 對新胞檢查
    候選機 = 可行構型且 (E^rem_i − e(剩餘計畫) − e(插入 Z_new) ≥ r·E_nom + e^alt) 者
    若候選 ≠ ∅: AUCTION 插入（邊際 makespan 最小；論文 4.3.2：先完成當前作業路徑）
    否則 若 multi_sortie: 派下一架次；否則回報 UNCOVERABLE(能量) 並保留於待辦
```

### 10.3 低電量／能量偏差（energy deviation）
遙測回報 $E^{tel}_i$（battery_monitor_validated=false 時僅作趨勢，門檻加嚴）；模型預測 $E^{pred}_i$。
```
on ENERGY_UPDATE(i, E_tel):
    δ = (E_pred − E_tel)/E_pred
    if δ > δ_1 (預設 0.05): 更新該機 k_unc,i ← k_unc,i·(1+δ)；重算剩餘計畫能量
    if E_tel − e(剩餘計畫含RTB) < r·E_nom:                     # 計畫不再可行
        cut ← 從計畫尾端逐條移除作業路徑直到可行；移除的路徑 → L_uncov → REPLAN 給其他機
    if E_tel < r·E_nom + e^alt(S_i, nearest alt) + margin:    # 即將觸底
        立即 RTB / 就近備降（DIVERT），該機所有剩餘路徑 → L_uncov → REPLAN
    if battery_monitor_validated=false and δ > δ_2 (0.15): 保守 DIVERT
```

### 10.4 失鏈（link loss）
機載預載策略 `lost_link_policy ∈ {RTB, CONTINUE_TO_NEXT_WP, NEAREST_ALT}`（每機可不同；預設：A 在鏈路多邊形內 CONTINUE 至 $t_{ll}$=60 s 後 RTB；B RTB）。
```
on LINK_LOSS(i, t):
    地面：假定機 i 執行預載策略 → 以其計畫推算航跡；其餘機避開該推算航跡的高度層與時間窗（11 節）
    若 t − t_loss > t_ll: 視同 UAV_LOSS(i) 做 REPLAN（但保留其推算 RTB 走廊獨占）
    若恢復：以其實際狀態做 ENERGY_UPDATE，並把它「重新加入」拍賣（未覆蓋路徑重新競標）
```

### 10.5 風場更新（wind update）
```
on WIND_UPDATE(W'):
    for 每航段（含 RTB）: 重算 V_g、t、e；作業方向偏好可能反轉（t^LR/t^RL 對調）
    if 任一機 ENERGY_SAFE_CHECK 失敗 → 10.3 的 cut 流程
    if max ΔC_i > τ_w (預設 5%) 或 掃描方向候選的總時間變化 > 10%：
        對尚未開始的胞重跑方向選擇（5 節第 3 項）與 H-IDP；已開始的胞只重跑剩餘路徑（保留條帶）
    if W' > W_abort 任一層：WEATHER_ABORT
    轉換段：若地面站風 > 轉換風限，延後降落（loiter 於基地上風側，需能量允許）或改備降點
```

### 10.6 目標線索 → 派另一構型近距確認（cue-and-confirm）
```
on TARGET_CUE(p, src=i_A, conf):
    建立確認胞 Q_conf: 以 p 為中心、半徑 r_conf（預設 150 m）的圓 → 8 邊形；σ = rgb；val = val_high；
        GSD_req = gsd_confirm（預設 2 cm）→ h_oper^B；時間窗 [t_now, t_now + t_conf_max]
    候選 = {j ∈ U^B : Q_conf ⊂ P_link, 插入後能量可行, 插入後 C_j 增量最小}
    if 候選 ≠ ∅: 以 AUCTION 插入 j 的計畫（可插在當前路徑之後，優先於低 val 胞；makespan 允許增加 ≤ τ_cue）
        j 的確認路徑：單條帶（Q_conf 直徑 ≤ s^B 時）或 2–3 條帶；亦可選 orbit(R^B, 2 圈) 模式 `confirm_mode=orbit`
    elif A_has_rgb and 某 A 機能量允許: A 自行降高（受 h_min^A=120 限制 → GSD 受限）做確認
    else: 加入待辦（priority queue）；下一架次或機隊釋放時處理
    A 機不中斷其 LiDAR 掃描（避免破壞第一層覆蓋）
```

### 10.7 優先度變更（priority change）
```
on PRIORITY_CHANGE(k, val'):
    val_k ← val'；重算 J（僅 ω4 項變動）
    if 胞 k 尚未開始：拍賣重排（允許搶占：高 val 胞可插在低 val 胞之前，條件為能量可行）
    if 胞 k 進行中且 val 降低：不中斷（避免殘段）；只影響後續排序
```

### 10.8 氣象中止（weather abort）
```
on WEATHER_ABORT(scope ∈ {ALL, config c, region}):
    受影響機：完成當前作業路徑 若 (t_remaining_path < 60 s 且能量允許) 否則立即中止
    每機選擇降落點 = argmin_a { t(S_i → a) : a ∈ ALT ∪ {H0}, 該點風 ≤ 轉換風限 }
    起降走廊獨占排程（11 節）：同一降落點依 ETA 排序，間隔 t_sep_ldg（預設 90 s），後續機在上風側 orbit 等待（需能量）
    全部未覆蓋路徑 → 待辦；記錄以便氣象恢復後 REPLAN
```

---

## 11 防撞／去衝突

1. **高度分層**：A、B 各自作業層與轉移層（9.3）；同層僅同構型。
2. **同層時間分離**：對任兩機同層航段做 4D 掃描（$\delta t$=1 s），要求水平距離 $\ge d_{sep}$（預設 150 m $> R^A$ 之機動裕度）或時間差 $\ge t_{sep}$（30 s）；衝突時延後較低 $val$ 者的進入（在轉換完成點後插入直線延伸，不用懸停）或交換條帶方向（二元結構下零成本選項）。
3. **同胞多機**：IDP 中相鄰條帶同時作業的機保持「同向或反向錯位」——加入 IDP 剪枝：若 $\pi_i,\pi_j$ 相鄰（$|l_i-l_j|\cdot s^c<d_{sep}$）且時間重疊則剪枝。
4. **起降走廊獨占**：基地／備降點的起飛與降落走廊為互斥資源，起飛依序間隔 $t_{sep,TO}$（60 s），降落依 ETA 排序間隔 $t_{sep,LDG}$（90 s）；等待以上風側 orbit 實現並計入能量。

---

## 12 複雜度

- 矩陣：每構型每胞 $O(n_q^2)$ 條 Dubins（含二元 ×2），每條 $O(1)$；風積分離散 $O(n_q^2\,N_{arc})$。兩構型 → ×2。
- 配置：貪婪 $O(|\mathcal C|\log|\mathcal C|+|\mathcal C|\,n_U)$；拍賣 $O(|\mathcal C|^2 n_U\,\bar L)$（$\bar L$ 平均計畫長度），局部改善 $O(N_{ls}|\mathcal C|n_U)$。
- H-IDP：階段數 $\lceil n_q/\lambda\rceil$，每階段 $\beta\cdot A(n_{rem},\lambda)\le\beta\, n_q^{\lambda}$ 個排列，每排列 $O(\lambda)$ 查表 → $O(\beta\,n_q^{\lambda+1})$。實務 $n_q\le40$、$\lambda\le4$、$\beta\le5$：$5\cdot40^5\approx5\times10^8$ 查表上界，但因 $n_{rem}$ 遞減實際約 $\sum_s (n_q-\lambda s)^\lambda\approx n_q^{\lambda+1}/(\lambda(\lambda+1))$，即 $\sim2.5\times10^7$（Python+numpy 向量化 < 2 s；$\lambda\le3$ 時 < 0.2 s，與論文 Table 4 的 0.06–0.4 s 同量級）。$\lambda>4$ 時強制 $\beta=1$ 並以 skip-k 啟發限制候選轉移（只列舉 $|l'-l|\in\{k,k+1,1\}$ 的排列 → $O(3^\lambda)$）。
- 高度：$O(N_{pts})$ 兩趟 + 平滑迭代 $O(N_{pts}N_{gd})$。
- 動態事件：只重算受影響胞與第 0 列，典型 < 1 s。

---

## 13 與論文對照

| 論文元素 | 本設計 | 退化條件 |
|---|---|---|
| 同構固定翼、定速 $V$ | 兩構型、風三角地速、感測器速度上限 | $A\equiv B$、$W=0$ → 時間 = 距離/V |
| FOV 梯形 Eq.2–3 | LiDAR／相機模型；梯形保留為選項 | `fov_model=trapezoid` |
| 整體凸化 | 凸分割成胞、殘帶指派 | 胞數 = 1 時等價 |
| 掃描方向 = 最小寬度 | 候選方向以含風總時間評估 | $W=0$ → 最小寬度方向 |
| 三類路徑二元距離 Eq.10–12 | 二元 (時間, 能量)，新增返航段 | $t^{RTB}=0$、$V$ 常數 |
| $D_{i,k},E_{i,k}$ Eq.13–14；約束 19–26 | $\mathcal T,\mathcal E$ 與同一 $E$ 結構；約束逐字沿用 | 恆等 |
| $J_k$ Eq.30（4 項各 0.25） | 6 項（+返航時間、+適配度），時間版 | $\omega_5=\omega_6=0$ |
| $\lambda_k$ Eq.31 | 每構型獨立 Eq.31 + 能量下限 | 單構型、$E=\infty$ |
| Algorithm 1 貪婪 | 貪婪 + 拍賣 + min-max 局部改善 + 多架次 | 跳過 Phase 2/3 |
| Eq.33 相對增量成本 | makespan 下界 + tie-break + 相對增量（$\mu$） | 前兩項 0、$\mu=1$ |
| Algorithm 2 IDP | H-IDP：beam、能量剪枝、返航、跨胞串接 | $\beta=1$、無剪枝 |
| Algorithm 3 高度下降 Eq.35–39 | 坡度受限包絡（雙向）+ 同樣平滑／刪點；起降走廊 | 地形平緩時等價 |
| 動態事件：UAV 退出、新區域 | 8 類事件，含 cue-and-confirm 跨構型 | 僅前兩類 |
| 無能量／鏈路／空域／氣象 | 3.4–3.5、4.2、6.1 硬限制 | 全部設為無限 |

論文中 $h=1200$ m、$w=2200$ m、$V=230$ m/s、$R_{min}=4000$ m 之數值不適用本案；本案 $h\in[100,150]$ m、$w\in[140,170]$ m、$V\in[19,21.5]$ m/s、$R\in[120,200]$ m，**轉彎長度與條帶間距同量級**，故 7.3 節的燈泡轉彎與 skip-k 順序成為主要效率來源，是論文（$R\ll$ 條帶長且 $w\ll R$ 不成立）未遇到的情境。

---

## 14 參數表

### 14.1 表 1 全部鍵（值不變；使用方式）

| 鍵 | Pioneer PNP | HERO PNP | 本設計用途 |
|---|---|---|---|
| platform_type | lift_and_cruise_vtol_4plus1 | 同 | 狀態機 3.1 |
| wingspan_m | 3.20 | 2.18 | 起降淨空校核（$\ge$ 翼展/2 + 裕度） |
| fuselage_length_m | 1.68 | 1.14 | 部署（不入模型） |
| wing_area_m2 | 1.00 | 0.53 | 翼載 → 失速／轉彎交叉檢查 |
| mtow_kg | 24.9 | 7.0 | H：$m_i\le$ |
| reference_auw_kg | 24.6 | 6.65 | $m^{ref}_c$ |
| reference_payload_kg | 5.0 | 0.6 | 基準能耗 |
| payload_max_kg | 5.0 | <1.0 | 輸入檢查 |
| cruise_airspeed_mps | 21.5 | 19.0 | $V_a^c$ |
| cruise_airspeed_kmh | 77.4 | 68.4 | 顯示 |
| min_mission_airspeed_mps | 18.0 | 15.0–16.0（規劃用 16.0，下限 15.0） | 作業降速下限 H5 |
| transition_airspeed_mps | 16.0 | 14.0 | 轉換段、風限 |
| stall_airspeed_mps | 14.0 | 12.0 | 轉換風限裕度 |
| max_airspeed_mps | 35.0 | 待實機確認（預設 = 1.4·V_a，標示待實測） | 不用於巡航 |
| max_climb_angle_deg | 3.5 | 3.5 | 緊急上限（不規劃） |
| planning_climb_angle_deg | 3.0 | 3.0 | $\gamma^{\uparrow}$ |
| max_descent_angle_deg | 5.0 | 5.0 | 緊急上限 |
| planning_descent_angle_deg | 4.0 | 4.0 | $\gamma^{\downarrow}$ |
| max_fw_climb_rate_mps | 1.13 | 1.00 | 交叉檢查 $V_a\sin\gamma$ |
| max_fw_descent_rate_mps | 1.50 | 1.33 | 交叉檢查 |
| max_bank_angle_deg | 20 | 25 | $R=V^2/(g\tan\phi)$ 交叉檢查（A: 130 m, B: 79 m ≈ 理論值） |
| turn_radius_theoretical_m | 130 | 75 | 下限（不規劃） |
| turn_radius_planning_m | 200 | 120 | $R^c$ |
| vtol_site_clear_radius_m | 8 | 6 | 起降點檢查（暫定） |
| obstacle_horizontal_buffer_m | 30 | 25 | 起降點檢查（暫定） |
| transition_start_agl_m | 60 | 50 | $h^{ts}$（暫定，待實測） |
| transition_complete_agl_m | 80–100 | 70–90 | $h^{tc}$：幾何用下緣、能量用上緣（暫定） |
| min_fw_operation_agl_m | 120 | 100 | $h^{min}_c$ |
| wind_resistance_public | Beaufort 5 | Beaufort 5 | 僅參考；$W_{go}$ 另設 |
| reference_endurance_min | 176 | 136 | $T^{ref}_c$ |
| reference_total_range_km | 242 | 144 | 交叉檢查 |
| reserve_fraction | 0.30 | 0.30 | $r$ |
| usable_range_fraction | 0.55–0.60 | 0.55–0.60 | $f_u$（回推 $\eta_m$） |
| usable_distance_km | 133–145 | 79–86 | $D^{use}$ 交叉檢查 |
| symmetric_mission_radius_km | 66–72 | 39–43 | H2 半徑（保守用下緣） |
| airspeed_sensor_validated | false | false | $k_{as}=1.05$；轉換風限加嚴 |
| battery_monitor_validated | false | false | $k_{bm}=1.15$；$D^{use}$ 取下緣；10.3 門檻加嚴 |
| c2_link_range_km | 待填入 | 待填入 | $\rho^{link}$（預設 15 km，待實測） |
| terrain_clearance_m | 待任務定義 | 待任務定義 | 9.1 floor（預設 30 m） |
| alternate_landing_sites_required | true | true | 硬性：無備降點不放飛 |

### 14.2 新增可設定參數（預設值；標示待實測者需覆寫）

| 參數 | 預設 | 說明 |
|---|---|---|
| E_nom_Wh (A/B) | 待填入 | 標稱電池能量；缺省時以 SoC 單位運算 |
| f_dep | 1.0 | 基準續航耗用比例 |
| hover_power_ratio $k_h$ | 5.0（待實測） | $P_h/P_c^{ref}$ |
| t_tr_out / t_tr_in | 20 / 25 s（待實測） | 轉換時長 |
| v_z_up / v_z_dn | 2.5 / 1.5 m/s | 垂直起降速度 |
| η_p, η_idle | 0.6, 0.3 | 推進效率、下降怠速比 |
| k_bm, k_as | 1.15, 1.05 | 未驗證保守係數 |
| κ_load | 0.85 | 單機負載上限比例 |
| lidar_fov_usable_deg, PRR, η_ret, ρ_req | 70°, 240 kHz, 0.5, 20 pts/m² | LiDAR 需求 |
| camera f, S_w, S_h, p_pix | 16 mm, 23.5 mm, 15.6 mm, 3.9 µm | 相機示例 |
| o_side, o_fwd, f_shutter | 0.20（LiDAR）/0.30（相機）, 0.70, 1 Hz | 重疊與快門 |
| A_has_rgb | false | A 是否具 RGB 快視 |
| h_oper_A / h_oper_B | 150 / 100 m AGL | 預設作業高度（$\le h^{ceil}$） |
| h_ceil | 120 m AGL | 法規上限（可設） |
| W_go, G_go, W_abort | min(8.0, 0.45·V_a), 12, 10 m/s | 氣象門檻 |
| vis_go, T_min, T_max | 3 km, −10, 40 °C | |
| ω (J 權重) | (0.2,0.15,0.2,0.15,0.1,0.2) | 6.3 |
| ν (S 權重) | (0.15,0.15,0.20,0.20,0.20,0.10) | 6.2 |
| β (beam) | 3 | 8.4 |
| ε, μ, τ_0 | 0.05, 0.0, 60 s | 8.2 |
| r_conf, gsd_confirm, t_conf_max, τ_cue | 150 m, 2 cm, 15 min, 10% | 10.6 |
| t_swap, multi_sortie | 10 min, false | 6.6 |
| lost_link_policy, t_ll | A: CONTINUE→RTB@60 s；B: RTB | 10.4 |
| d_sep, t_sep, t_sep_TO, t_sep_LDG | 150 m, 30 s, 60 s, 90 s | 11 |
| δ_1, δ_2 | 0.05, 0.15 | 10.3 |
| δd, ε_DP | 25 m, 0.25·min s^c | 9.1, 5 |
| N_ls | 200 | 6.5 局部改善迭代 |
