# HARVEST 演算法設計（幾何／運動學／三維高度優先版）

Heterogeneous-VTOL Dynamic Complete Coverage Path Planning，兩構型：**A = Pioneer PNP（LiDAR）**、**B = HERO PNP（測繪相機）**，皆為 lift-and-cruise 4+1 複合翼。
本文件以 Wang et al. 2025（Defence Technology 47:197–215）為骨架，將每一層擴展為異構、含風、含地形坡度限制、含起降走廊的版本；著重幾何正確性（覆蓋完整性）與運動學可飛性（flyability）。

---

## 1 問題定義與假設

### 1.1 問題敘述

給定：
- 任務區域集合 $\mathcal{Z}=\{Z_k\}$（任意簡單多邊形，可凹），每區附**感測需求** $req_k\in\{\text{lidar},\text{rgb},\text{both},\text{any}\}$、優先度 $val_k$、可選時間窗。
- 機隊 $\mathcal{U}=\mathcal{U}_A\cup\mathcal{U}_B$，每架 $U_i$ 具構型 $c(i)\in\{A,B\}$、當架次 AUW、可用電量 $E_i$、起飛點 $H_i$。
- 備降點集合 $\mathcal{S}_{alt}$、DEM/DSM、障礙物集合 $\mathcal{O}$、高度分層風場 $\mathcal{W}$、C2 鏈路覆蓋 $\mathcal{C}$、空域限制（高度上限、禁區、時間窗）、氣象門檻。

求：每架 $U_i$ 一條**完整架次航跡** $\Gamma_i$ = 起飛走廊 → 進入路徑 → (作業路徑, 轉移路徑)* → 返航路徑 → 降落走廊，三維、時間參數化，使

$$\min \; T_{mission}=\max_i T_i(\Gamma_i)$$

受限於：(C1) 每區依 $req_k$ 被對應構型之感測器足跡**完全覆蓋**；(C2) 運動學可飛（曲率 $\le 1/R_{eff}$、地面坡度在 $[-\tan\gamma_d^g,\tan\gamma_c^g]$）；(C3) 高度在 $[z_{lo},z_{hi}]$ 包絡內（地形淨空、障礙淨空、最低固定翼高度、感測器最大高度、空域上限）；(C4) 能量：任一航點剩餘能量 $\ge$ 到最近備降點之 RTB 能量 + 30% 保留；(C5) 鏈路／任務半徑；(C6) 去衝突。

### 1.2 假設（承襲論文並修正）

| 論文假設 | 本設計 |
|---|---|
| 同構、定速 | 兩構型；每航段空速 $V_a\in[V_{a,\min},V_{a,c}]$ 可下調以滿足感測地速上限 |
| 起始位置在區外、不含起降 | 起始狀態 = 轉換完成點（或爬升盤旋出口）；含垂直起降與轉換走廊 |
| FOV 前視梯形固定 | LiDAR／相機足跡由**該條帶保守最小 AGL** 決定；論文梯形保留為選項 |
| 上升／下降定地速 | 3°/4° 為**空氣坡度**；地面坡度隨風修正（§9.1） |
| 區域整體凸化 | Douglas–Peucker 簡化後做**凸分割**（不整體凸化） |
| 無風、無障礙（僅圓形威脅） | 高度分層風場；點／線／面障礙 + 高度的 keep-out |
| 成本 = 距離 | 成本 = **時間**（風三角） |

其他假設：地面 ENU 平面（原點 = 主起飛點，WGS84→ECEF→ENU，含地球曲率，故不需論文 Eq.35 的曲率補償）；風場在規劃期間視為準靜態（更新觸發重規劃 §10）；感測器為下視（nadir）安裝，前視版本以論文 Eq.(2)(3) 替換 §3.2 即可。

---

## 2 符號表

| 符號 | 意義 |
|---|---|
| $c\in\{A,B\}$ | 構型；$A$=Pioneer PNP(LiDAR)，$B$=HERO PNP(相機) |
| $V_{a,c}, V_{a,\min,c}$ | 巡航空速、任務空速下限（21.5/18.0；19.0/15.5） |
| $R_{plan,c}, R_{theo,c}, R_{eff,c}$ | 規劃／理論／風修正有效轉彎半徑 |
| $\gamma_c^{air}=3^\circ,\ \gamma_d^{air}=4^\circ$ | 規劃爬升／下降空氣坡度 |
| $\gamma_c^{g}(\theta),\gamma_d^{g}(\theta)$ | 對應航跡方位 $\theta$ 之地面坡度 |
| $\vec W(h)$, $W$, $\beta$ | 風向量（高度分層）、風速、風來向 |
| $V_g(\theta,V_a,\vec W)$ | 航跡方位 $\theta$ 之地速 |
| $h$ / $h_{\min,l}$ | AGL；條帶 $l$ 的保守最小 AGL |
| $w_c(h), s_c(h)$ | 構型 $c$ 在 AGL $h$ 的足跡寬（swath）與條帶間距 |
| $o_{side},o_{fwd}$ | 側向／前向重疊率 |
| $Q_k$ | 凸胞（convex cell）；$Z_k=\bigcup_q Q_{k,q}$ |
| $\alpha_{k,q}$ | 胞的掃描方向（航跡方位） |
| $L_{k,l}$, $p^L_{k,l},p^R_{k,l}$ | 作業路徑（條帶中心線）及左右端點 |
| $S=(x,y,\psi,R)$ | Dubins 狀態；$D(S_m\to S_n)$ Dubins 曲線 |
| $\overset{\leftrightarrow}{t}^{oper},\overset{\leftrightarrow}{t}^{in},\overset{\leftrightarrow}{t}^{trans}$ | 二元（雙方向）作業／進入／轉移**時間** |
| $T_{i,k}, E_{i,k}$ | 時間矩陣（對應論文 $D_{i,k}$）、決策矩陣 |
| $z_g(x,y)$ | DEM 地表高（ENU z） |
| $z_{lo}(s),z_{hi}(s),z^*(s)$ | 高度包絡下界、上界、坡度受限最小可行剖面 |
| $H_{ts,c},H_{tc,c}$ | 轉換起始／完成 AGL（60/90；50/80，預設取區間中值） |
| $r_{site,c}, b_{obs,c}$ | 起降淨空半徑（8/6）、障礙水平緩衝（30/25） |
| $\mathcal{K}$ | keep-out 集合（含高度） |
| $\lambda_k$ | 指派給 $Z_k$ 之機數 |

---

## 3 平台／感測器／能量模型

### 3.1 運動學（論文 Eq.1 之擴展）

$$\dot x=V_g\sin\theta,\quad \dot y=V_g\cos\theta,\quad \dot z=V_a\sin\gamma^{air},\quad |\dot\theta|\le V_g/R_{eff}$$

ENU 中航跡方位 $\theta$ 為順時針自北起算，$\hat t=(\sin\theta,\cos\theta)$，$\hat n=(\cos\theta,-\sin\theta)$（右法向）。

**風三角**：風來向 $\beta$，$\vec W=-W(\sin\beta,\cos\beta)$；$W_\parallel=\vec W\cdot\hat t$，$W_\perp=\vec W\cdot\hat n$：

$$V_g(\theta)=\sqrt{V_a^2-W_\perp^2}+W_\parallel,\qquad \psi=\theta-\arcsin(W_\perp/V_a)$$

可行條件：$|W_\perp|<V_a$ 且 $V_g>V_{g,\min}$（預設 $V_{g,\min}=3$ m/s，避免逆風「原地不動」）。

**轉彎半徑**：$R_{theo}=V_a^2/(g\tan\phi_{\max})$（Pioneer：$21.5^2/(9.81\tan20^\circ)=129.5$ m ≈ 130；HERO：$19^2/(9.81\tan25^\circ)=78.9$ m ≈ 75，表 1 一致）。空氣圓在風中的地面軌跡為擺線，其**最大**曲率半徑（順風相位）為 $R(1+W/V_a)^2$。因此

$$R_{eff,c}(W)=\max\Big\{R_{plan,c},\ R_{theo,c}\big(1+W/V_{a,c}\big)^2\Big\}$$

$R_{plan}$ 內建的裕度可吸收 $W^*=V_a(\sqrt{R_{plan}/R_{theo}}-1)$：Pioneer 5.2 m/s、HERO 5.0 m/s；風速超過即放大。以地面半徑 $\ge R_{eff}$ 規劃之航跡在任何相位皆可以 $\phi\le\phi_{\max}$ 追蹤 → 可飛性（§7.6）。

**坡度**：$3^\circ/4^\circ$ 為空氣坡度；地面坡度 $\tan\gamma^g(\theta)=V_a\sin\gamma^{air}/V_g(\theta)$。保守取 $V_g$ 最大值（順風）：Pioneer, $W=8$：$\tan\gamma_c^g=0.0523\cdot21.5/29.5=0.0381$（2.18°）。表 1 的 max_fw_climb_rate 1.13 m/s $=21.5\sin3^\circ$ 一致，用於時間換算：$\dot z=V_a\sin\gamma$ 與風無關。

### 3.2 感測器足跡（取代論文 Eq.2/3；論文梯形保留為 `fov_model="trapezoid"`）

**A（LiDAR）**，可用掃描角 $\theta_{use}$（預設 70°，可設）、脈衝率 $f_p$、需求點密度 $\rho_{req}$（pt/m²）、最大測距 $r_{\max}$：

$$w_A(h)=2h\tan(\theta_{use}/2),\qquad s_A=w_A(1-o_{side,A}),\qquad V_g^{cap,A}=\frac{f_p}{w_A\,\rho_{req}},\qquad h\le h_{\max,A}=r_{\max}\cos(\theta_{use}/2)\cdot k_{rng}$$

**B（相機）**，感光元件寬高 $S_w,S_h$、焦距 $f$、像元 $p$、快門率 $f_s$、需求 GSD：

$$w_B(h)=h\frac{S_w}{f},\quad l_B(h)=h\frac{S_h}{f},\quad GSD=h\frac{p}{f},\quad h_{\max,B}=\frac{GSD_{req}f}{p},\quad s_B=w_B(1-o_{side,B}),\quad V_g^{cap,B}=l_B(1-o_{fwd})f_s$$

**共同**：目標 AGL $h_{nom,c}=\mathrm{clip}(h_{sensor}^{target},\,h_{\min fw,c},\,h_{\max,c})$，$h_{\min fw}$=120/100 m AGL。條帶寬永遠以 $h_{\min,l}$（該條帶沿線最小 AGL，§9.3）代入 $w_c(\cdot)$ → 足跡在任一點 $\ge w_c(h_{\min,l})$ → 覆蓋保證。端點延伸 $e_{ext,c}=\max(V_{g}\,t_{settle},\ l_B(1-o_{fwd}))$（滾轉安定 $t_{settle}$ 預設 2 s；相機需第一張影像前向重疊）。

### 3.3 速度選擇（地速上限與空速下限之交集）

每航段給定 $\theta$：$V_a^*=\max\{V_a: V_a\in[V_{a,\min},V_{a,c}],\ V_g(\theta,V_a)\le V_g^{cap}\}$；若 $V_g(\theta,V_{a,\min})>V_g^{cap}$ → 該方向**不可行**（成本 $\infty$），IDP 自然選另一方向；兩方向皆不可行 → 胞改掃描方向或提高 $f_s$。轉移／進入／返航段不受 $V_g^{cap}$ 限制，用 $V_{a,c}$。

### 3.4 能量模型（幾何相關部分；細節由能量代理定義）

以「能量分率」表示：$\dot f_c=(AUW/AUW_{ref})^{1.5}/t_{ref}$（巡航每秒消耗分率；$t_{ref}$=176/136 min），$\dot f_h=k_h\dot f_c$（懸停，$k_h$ 預設 4.0，待實測），轉換段以 $\dot f_h$ 計時 $t_{tr}$，爬升段乘 $(1+k_{climb}\sin\gamma/\sin3^\circ)$（$k_{climb}$ 預設 0.35）。可用分率 $f_{use}=(1-0.30)\cdot k_{bm}$，$k_{bm}=0.9$ 若 battery_monitor_validated=false。以距離表示的驗算：usable_distance 133–145 / 79–86 km 與 usable_range_fraction 0.55–0.60 × total_range 相符。航點能量檢查：

$$f_{rem}(j)\ \ge\ f_{RTB}(j\to \text{nearest alt})+0.30,\quad f_{RTB}=\text{Dubins 返航(含逆風)}+\text{下降}+\text{轉換}+\text{垂直降落}$$

---

## 4 任務輸入與前置檢查

表 2 全部輸入映射：起飛／備降點（WGS84→ENU；每點含 $r_{site}$ 淨空與 $b_{obs}$ 檢查）；任務區域 GeoJSON（每區附 $req_k$）；DEM/DSM；障礙物（點/線/面+高）；風場分層；當架次 AUW；可用電量；酬載需求（$\theta_{use},f_p,\rho_{req}$ 或 $f,S_w,S_h,p,f_s,GSD,o_{side},o_{fwd}$）；C2 覆蓋（km 或多邊形）；空域（$h_{ceil}$、禁區、時間窗）；氣象門檻。

```
PRECHECK(inputs):
  # 氣象 go/no-go
  assert W_mean <= W_go_c, W_gust <= W_gust_c, vis >= vis_min, rain <= rain_max, T in [T_min,T_max]
  assert crosswind_at_site(H) <= X_wind_max_c            # 轉換段側風
  # 幾何一致性
  for each site in {H} ∪ S_alt: assert clear_radius_free(site, r_site_c) and no_tall_obstacle_within(site, b_obs_c)
  assert h_ceil - h_minfw_c >= dh_band_min                 # 高度帶非空（Pioneer 120 AGL vs 法規 120 AGL → 需豁免，否則拒絕）
  assert every vertex of every Z_k inside C2 coverage and inside symmetric radius r_sym_c (or link polygon)
  # 感測需求可達性
  for Z_k: if req_k in {lidar, both}: need U_A nonempty ; if req_k in {rgb, both}: need U_B nonempty
  # 未驗證航電之保守化
  if not airspeed_sensor_validated: V_tr_c += dV_tr_margin ; H_tc_c = upper bound of range
  if not battery_monitor_validated: k_bm = 0.9
  if c2_link_range_km is None or terrain_clearance_m is None: raise MissingInput   # 表 1「待填入」
  return derived params {R_eff_c(W_h), gamma_g_c(theta), h_nom_c, w_c, s_c, V_g^cap_c}
```

---

## 5 區域分解（幾何核心）

### 5.1 總流程

```
DECOMPOSE(Z_k, req_k, params):
  P  = DouglasPeucker(Z_k, eps_dp)                      # eps_dp = 0.1 * min_c s_c
  P  = P ⊕ disc(delta_edge)  # 邊界外擴 delta_edge（§5.6），避免定位誤差漏覆蓋
  Q  = ApproxConvexDecomp(P, tau = 0.5 * min_c w_c)     # §5.2
  for Q_q in Q:
      C_q = candidate_sweep_directions(Q_q)              # §5.3
      for c in types_allowed(req_k):
          for theta in C_q:
              strips[c,theta] = lay_strips_conservative(Q_q, theta, c)   # §5.4 + §9.3 回饋
              T[c,theta]      = proxy_time(strips[c,theta], theta, c)    # §5.3
          theta*_c = argmin_theta T[c,theta]
      residual_handoff(Q_q)                              # §5.5
  return cells with per-type strip sets
```

### 5.2 凸分割（不整體凸化）

Douglas–Peucker 容差 $\varepsilon_{dp}=0.1\min_c s_c$（簡化誤差遠小於條帶間距，且簡化後**只允許外擴**：對每個被刪除的頂點，若其位於簡化邊內側則保留簡化，若位於外側則以該頂點替代——即取「簡化多邊形 ∪ 原多邊形」的邊界，保證 $Z_k\subseteq P$）。

凸分割採 **Hertel–Mehlhorn**（耳切三角化 + 移除非必要對角線，$\le 4\times$ 最優片數，$O(n\log n)$）後接 **近似凸分解合併**（Lien–Amato ACD 準則）：兩相鄰胞可合併若合併後的凹度 $\kappa(Q_a\cup Q_b)\le\tau$，$\tau=0.5\min_c w_c$。理由：深度小於半個足跡的凹口，由論文 Eq.(8)(9) 的「條帶內頂點延伸端點」即可覆蓋，不必獨立成胞。

```
ApproxConvexDecomp(P, tau):
  T = ear_clipping_triangulation(P)
  Q = hertel_mehlhorn_merge(T)                     # 只合併使結果仍凸的對角線
  repeat:
     best = argmax over adjacent (Qa,Qb) of  merge_gain(Qa,Qb)  s.t. concavity(Qa ∪ Qb) <= tau
     if best is None: break
     merge(best)
  # merge_gain = 減少的轉場數估計 = n_strips(Qa)+n_strips(Qb)-n_strips(Qa∪Qb) 之最佳方向值
  return Q
```

合併後的胞可能「近凸」（凹度 $\le\tau$）；條帶端點公式 §5.4 對近凸胞仍正確，因為端點取「條帶帶內裁剪子多邊形」的 $u$ 極值，與凸性無關。

### 5.3 候選掃描方向與風感知評估

對凸胞 $Q_q$（頂點 $v_m$）：候選集 $C_q=\{$各邊方向 $\theta_m\}\cup\{\theta_{minw}\}\cup\{\beta\pm90^\circ\}$。$\theta_{minw}$ = 最小寬度方向（論文：對每邊求最遠頂點距離 $dis_{k,mu}$，$w_{k,\min}=\min_m\max_u dis_{k,mu}$，對應邊方向），為主要候選（條帶數最少）。$\beta\pm90^\circ$（側風方向）為風感知候選：直線來回一對之時間因子，順逆風 $=\frac{2L V_a}{V_a^2-W^2}$，側風 $=\frac{2L}{\sqrt{V_a^2-W^2}}$，後者恆較小。

**代理時間**（proxy；IDP 前的快速評估，含風、含轉場、含地形坡度回饋）：

$$T_{proxy}(\theta,c)=\sum_{l}\Big[\tfrac{1}{2}\big(\tfrac{|L_l|}{V_g^*(\theta)}+\tfrac{|L_l|}{V_g^*(\theta+\pi)}\big)\Big]+(n_l-1)\,t_{turn}(s_{l},R_{eff,c})+\text{pen}_{slope}$$

其中 $t_{turn}$ 取 §7.3 三種轉場最小者（間距 $s_l<2R_{eff}$ 時自動使用 skip-$k$ 的 U 型時間），$\text{pen}_{slope}=k_s\sum_l \max(0,\ \overline{|\partial_s z_g|}_l-\tan\gamma_c^g)\cdot|L_l|/V_g$：沿航跡的地形坡度超過可爬升坡度時，AGL 將被迫放大（§9.2）、足跡寬縮小、條帶數增加；此懲罰把「沿等高線掃描」的偏好內建於方向選擇。取 $T_{proxy}$ 前 3 名再以精確 §5.4（含 §9.3 回饋）重算，選最小。

### 5.4 條帶產生（sensor-driven, conservative-AGL）

掃描座標系：$u$ 沿 $\hat t(\theta)$，$v$ 沿 $\hat n(\theta)$，原點取胞在 $v$ 方向的最小點（基準邊）。胞的掃描寬 $W_q=v_{\max}-v_{\min}$。

**序列鋪設**（每條條帶的寬度由自身的保守 AGL 決定，下一條自本條上緣減去重疊起算）：

```
lay_strips_conservative(Q, theta, c):
  v_base = v_min(Q); l = 1; strips = []
  while v_base < v_max(Q):
      h = h_nom_c
      repeat (max 3 iterations):
          w  = w_c(h)                              # 足跡寬
          band = {v_base <= v <= v_base + w}
          sub = clip(Q, band)                      # 帶內裁剪子多邊形（凸∩帶 = 凸）
          if sub is empty: break
          u_lo = min_u(sub) - e_ext_c ; u_hi = max_u(sub) + e_ext_c
          centerline: v_c = v_base + w/2 , u in [u_lo, u_hi]
          prof = local_slope_envelope(centerline, c)          # §9.2，僅此條帶＋兩端 R_eff 延伸
          h_min = min_s ( prof.z*(s) - z_g^max(s) )           # 走廊內最高地形之上的最小 AGL
          if h_min >= h - eps_h: break
          h = h_min                                            # 縮寬重鋪（單調遞減 → 收斂）
      strips.append(L_l = {pL=(u_lo,v_c), pR=(u_hi,v_c), w, h_min, v_lo=v_base, v_hi=v_base+w})
      v_base = v_base + w - w*o_side_c            # 下緣 = 上緣 − 重疊  ⇒ 無縫
      l += 1
  # 最後一條溢出量 Delta = v_hi(last) - v_max(Q)；若 Delta > 0 ，均分縮距（保留條數）：
  #   s' = (W_q - w_last)/(n-1)（僅當 s' >= s_min_c，否則維持）
  return strips
```

對應論文 Eq.(6)(7)：$L^T_l(q)=v_{k,2}+q\hat u+\Delta w^T_l\hat n$，$\Delta w^T_l=(l-1)(w-w_{ovl})$；本設計因 $w$ 逐條變動，$\Delta w^T_l=\sum_{j<l}(w_j-w_{ovl,j})$。端點對應 Eq.(8)(9)：$p^L=\min_u\{c^{LT},c^{LB},\{v_m:I_{k,l}^{v_m}=1\}\}$ 即 `min_u(sub)`，本設計以裁剪子多邊形直接取極值，並外加 $e_{ext}$。

**走廊地形**：$z_g^{\max}(s)=\max_{|v-v_c|\le w/2}z_g(s,v)$（DEM 雙線性 Eq.4 於橫向取樣 $\Delta v=\min(10\text{ m},w/8)$）。

### 5.5 殘帶指派（residual band handoff）

胞 $Q$ 由 A 鋪設 $n_A$ 條後，最後一條實際只需覆蓋殘寬 $r=W_q-\big(v_{hi}(n_A-1)-o_{side}w_{n_A-1}\big)$。若 $req_k=\text{any}$ 且 $r<w_A$：

$$\text{選 B 接手} \iff t_{strip,A}(L_{n_A})+t_{turn,A} \;>\; \sum_{j=1}^{n_B^{res}}t_{strip,B}+ (n_B^{res}-1)t_{turn,B}+t_{detour,B},\quad n_B^{res}=\lceil (r-w_B)/s_B\rceil^+ +1$$

殘帶多邊形 $Q^{res}=\mathrm{clip}(Q,\{v\ge v_{hi}(n_A-1)-o_{side}w\})$ 外擴 $\delta_{edge}$ 後成為 B 的獨立小胞，掃描方向沿用 $\theta$（或重評）。$req_k\in\{\text{lidar},\text{both}\}$ 不允許（LiDAR 需求不能以相機替代）。$t_{detour,B}$ 為 B 到殘帶的額外轉移估計（取 B 現有最近胞的 Dubins 時間）。

### 5.6 胞邊界縫隙防止與覆蓋完整性論證

1. **分割精確性**：$\{Q_q\}$ 為 $P$ 的精確分割（三角化＋合併不改變並集），$Z_k\subseteq P$。
2. **胞內完整**：條帶帶 $[v_{lo,l},v_{hi,l}]$ 序列自 $v_{\min}$ 起、相鄰帶重疊 $o_{side}w$，至 $v_{hi}\ge v_{\max}$ 止 → 帶覆蓋 $[v_{\min},v_{\max}]$。每帶內，中心線 $u\in[u_{lo},u_{hi}]$ 涵蓋 `sub` 的 $u$ 極值，足跡矩形 $[u_{lo},u_{hi}]\times[v_{lo},v_{hi}]\supseteq \text{sub}$。足跡實際寬 $w_c(h(s))\ge w_c(h_{\min,l})=w$（$w_c$ 對 $h$ 單調遞增，且 §9.3 保證最終剖面 AGL $\ge h_{\min,l}$）→ 每帶內完全覆蓋。
3. **胞間縫隙**：相鄰胞掃描方向不同時，兩胞足跡在共同邊界處各自恰好到邊界；定位誤差 $\sigma_{xy}$、航跡追蹤誤差 $e_{xt}$、DEM 誤差引起的足跡寬誤差 $\delta_w=w\,\sigma_h/h$ 可能造成縫隙。對策：§5.1 將 $P$ 外擴 $\delta_{edge}=k_\sigma\sigma_{xy}+e_{xt}+\delta_w/2$（預設 15 m），且每個胞在分割後**各自**再外擴 $\delta_{edge}$（胞互相重疊 $2\delta_{edge}$）。凸胞外擴仍凸。
4. **DEM 保守性**：以走廊最高地形計 AGL → 任一橫向點的真實 AGL $\ge h_{\min}$。
結論：$\bigcup_{l,q}\text{footprint}(L_{q,l})\supseteq\bigcup_q Q_q\oplus\delta_{edge}\supseteq Z_k$。

### 5.7 胞的構型適配度（feed §6）

$$\phi_{q,c}=g_{req}\cdot\Big[\omega_1\min\!\big(1,\tfrac{W_q}{w_c}\big)^{-1}\!\!\cdot\!\mathbb{1}[W_q<w_c]+\omega_2\,\tfrac{n_{turn,q,c}\,t_{turn,c}}{T_{proxy,q,c}}+\omega_3\,\tfrac{d(H,Q_q)}{r_{sym,c}}+\omega_4\,\mathbb{1}[w_{q,\min}<2R_{eff,c}]\Big]^{-1}$$

$g_{req}$：感測門檻（不符 $req_k$ → 0）。項 1：胞窄於足跡（浪費）；項 2：轉場時間占比（窄長胞對大半徑不利）；項 3：距離／續航半徑比；項 4：胞最小寬度小於 $2R_{eff}$ 時無法在胞內做 U 型轉場（需 skip 或伸出胞外）。A 偏好大、遠、寬胞；B 偏好小、近、窄、凹角區。

---

## 6 任務配置（摘要；重點在幾何量的來源）

沿用論文 Algorithm 1 之貪婪配置，指標擴充：

$$J_{k,c}=\omega_1\frac{\sum_i\min_l t^{in}_{i,k,l}}{\sum_j\sum_i\min_l t^{in}_{i,j,l}}+\omega_2\frac{n_{k,c}}{\sum_j n_{j,c}}+\omega_3\frac{T^{oper}_{k,c}}{\sum_j T^{oper}_{j,c}}+\omega_4\frac{val_k}{\sum val}+\omega_5\,\bar\phi_{k,c}$$

（$\omega_{1..4}=0.2$，$\omega_5=0.2$；退化：$\omega_5=0$、單構型 → 論文 Eq.30）。$\lambda_{k,c}=\mathrm{round}(n_{U_c}J_{k,c}/\sum_j J_{j,c})$。硬限制：能量可行（含 RTB）、鏈路／半徑、感測需求、時間窗。多架次（換電再出）為選項。$req=\text{both}$ 的區對 A、B 各產生獨立條帶集（高度分層 §11）。配置代理另有細節；本文件要求配置輸出 $\{(U_i, \text{cell list}, \text{type})\}$ 供 §8 使用。

---

## 7 三類路徑成本（時間、含風、Dubins）

### 7.1 Dubins 狀態與時間積分

狀態 $S=(x,y,\psi_{trk},R_{eff,c})$（以航跡方位表示，autopilot 自行偏流）。任兩狀態間取 6 種 word（LSL, RSR, LSR, RSL, RLR, LRL）最短者 $D(S_m\to S_n)$，並做 keep-out 檢查（§7.5）。**時間**：

$$t(D)=\int_D\frac{ds}{V_g^*(\theta(s))}\ \approx\ \sum_{\text{直線}}\frac{L}{V_g^*(\theta)}+\sum_{\text{弧}}\sum_{j}\frac{R_{eff}\Delta\theta}{V_g^*(\theta_j)}$$

弧以 $\Delta\theta=5^\circ$ 離散積分。爬升／下降對沿航速度之影響 $\cos3^\circ=0.9986$ 忽略。

### 7.2 二元成本（對應論文 Eq.10–12，距離→時間）

作業路徑 $L_{k,l}$，左右端點 $p^L,p^R$，方向 $\theta_{LR},\theta_{RL}=\theta_{LR}+\pi$：

$$\overset{\leftrightarrow}{t}^{oper}_{i,k,l}=\begin{bmatrix}t^{LR}\\ t^{RL}\end{bmatrix}=\begin{bmatrix}|L_{k,l}|/V_g^*(\theta_{LR})&,\ e^{oper}=-1\\ |L_{k,l}|/V_g^*(\theta_{RL})&,\ e^{oper}=+1\end{bmatrix}$$

$$\overset{\leftrightarrow}{t}^{in}_{i,k,l}=\begin{bmatrix}t\big(D(S_i\to S^L_{k,l})\big)\\ t\big(D(S_i\to S^R_{k,l})\big)\end{bmatrix},\qquad \overset{\leftrightarrow}{t}^{trans}_{i,k,l,l+s}=\begin{bmatrix}t\big(D(S^{LR}_{k,l}\to S^{L}_{k,l+s})\big)\\ t\big(D(S^{RL}_{k,l}\to S^{R}_{k,l+s})\big)\end{bmatrix}$$

其中 $S^L_{k,l}=(p^L,\theta_{LR},R_{eff})$（自左端點向右進入），$S^{LR}_{k,l}=(p^R,\theta_{LR},R_{eff})$（向右掃完後在右端點的狀態），依此類推。$S_i$ = 起飛走廊出口狀態（§9.5）；若使用爬升盤旋，$S_i$ 為盤旋切線點族 $\{S_i(\vartheta)\}$，$t^{in}$ 取 $\min_\vartheta$（同時滿足爬升距離條件）。矩陣 $T_{i,k}$、$E_{i,k}$ 之結構同論文 Eq.(13)(14)，Eq.(15)(16) 的二元加法規則不變；限制 Eq.(19)–(26) 原樣沿用（方向交替、進入不同路徑、每路徑一次）。作業方向若 §3.3 不可行則該元素 $=\infty$。

### 7.3 轉場幾何：相鄰間距 $s<2R$ 的處理

同一胞相鄰條帶間距 $s_l$；Pioneer $R_{eff}\ge200$ m 而 LiDAR 於 120 m AGL、70° 時 $w_A=168$ m、$s_A\approx134$ m $<2R=400$ m；HERO 於 100 m AGL、24 mm/36 mm 幅寬時 $w_B=150$ m、$s_B=105$ m $<240$ m。**幾乎所有情況都 $s<2R$**，故轉場圖樣：

| 圖樣 | 條件 | 長度 | 超出端線 | 側向溢出 |
|---|---|---|---|---|
| U 型（CSC） | $ks\ge2R$ | $\pi R+(ks-2R)$ | $R$ | 0（在兩條帶之間） |
| 燈泡／Ω 型（RLR） | $s<2R$ | $R(\pi+4\theta_\Omega)$，$\theta_\Omega=\arccos\frac{s+2R}{4R}$ | $\sqrt{4R^2-(R+s/2)^2}+R$ | $R-s/2$（兩側） |
| 環型／loop（LSL） | $s<2R$ | $2\pi R+2R-s$ | $R$ | $2R-s$（單側） |

推導（Ω）：起點 $(0,0)$ 航向 $+u$ 右轉圓心 $O_1=(0,-R)$，終點 $(0,s)$ 航向 $-u$ 右轉圓心 $O_3=(0,s+R)$，中間左轉圓與二者相切 $|O_1O_2|=|O_2O_3|=2R$ → $\cos\theta_\Omega=(s+2R)/(4R)$，三弧角 $\theta_\Omega,\ \pi+2\theta_\Omega,\ \theta_\Omega$。$s=0$：$7.33R$；$s=R$：$6.03R$；loop 同條件 $8.28R,\ 7.28R$，Ω 較短但側向溢出對稱。**Skip-$k$**：$k=\lceil2R_{eff}/s\rceil$（無風下 A 典型 $k=4$，B $k=3$），序列如 $1,4,7,\dots$ 再回填，U 型長度 $\pi R+(ks-2R)$，且轉場區只在端線外 $R$ 之內、不側向溢出。

實作上：**不硬編碼圖樣**；轉移矩陣 $\overset{\leftrightarrow}{t}^{trans}_{l,l+s}$ 對所有 $s$ 由一般 6-word Dubins 求得（相鄰 $s=1$ 自動得到 Ω/loop，$s=k$ 自動得到 U 型），IDP 依時間自動選出 skip-$k$ 序列或局部 Ω；上表公式用於分析、複雜度上界與單元測試。每個轉場的**掃掠足跡**（路徑 ⊕ 半寬 $b_{trk}$）須通過 keep-out 與禁區檢查（§7.5），不通過者 $=\infty$。胞邊界外 $R_{eff}+e_{ext}$ 的「迴轉帶」（turn-around zone）在 §5 分解時即檢查是否落入禁區；若是，該方向的 $T_{proxy}$ 加大懲罰。

### 7.4 高度差驅動的轉移延長

相鄰作業路徑目標高度差 $\Delta z$（地形不同、或 $req$=both 時 A/B 分層）需水平距離 $\ge\Delta z/\tan\gamma^g$。若 Dubins 轉移長度不足：

$$t^{trans}\leftarrow\max\Big(t(D),\ \frac{|\Delta z|}{V_a\sin\gamma^{air}}\Big),\quad\text{幾何：在下一條帶入口前插入 } n_{lap}=\Big\lceil\frac{|\Delta z|-|D|\tan\gamma^g}{2\pi R_{eff}\tan\gamma^g}\Big\rceil^+\text{ 圈螺旋}$$

螺旋圓心置於迴轉帶內、通過 keep-out 檢查。

### 7.5 Dubins 離散化（autopilot）與 keep-out 檢查

**離散化**（論文 Eq.28/29 之外切多邊形法，改以最大橫向偏差 $e_{\max}$ 決定角步）：
$\Delta\psi=2\arccos(1-e_{\max}/R_{eff})$（$e_{\max}$ 預設 3 m → Pioneer 19.9°、HERO 25.7°；再取 $\min(\Delta\psi,\psi_{\max}=30^\circ)$）。切點 $P_{c,j}=O+R[\hat u_{OP},\hat u^\perp_{OP}][\cos(j\Delta\psi),\ \mathrm{sgn}\sin(j\Delta\psi)]^T$（Eq.28），航點 $P_{a,j}$ 取相鄰切線交點（Eq.29），使航點多邊形外切於弧、autopilot 的 L1 追蹤切入弧內。若 autopilot 支援弧命令（ArduPilot `NAV_LOITER_TO_ALT`/PX4 orbit）則直接輸出圓心、半徑、方向，僅直線段輸出航點。每航點附：$z$（§9）、$V_a^*$、感測器開關（作業路徑端點 ± $e_{ext}$）、接受半徑 $r_{acc}=\min(R_{eff}/2, 30\text{ m})$。

**keep-out**：每障礙物 $o$ 生成 $\mathcal{K}_o=(\text{poly}_o\oplus\text{disc}(b_{h,o}),\ z_{top,o}+b_{v})$；點→圓、線→膠囊、面→圓角偏移多邊形。檢查：航跡樣本 $(x,y,z)$ 若 $(x,y)\in\mathcal{K}_o$ 且 $z<z_{top,o}+b_v$ → 衝突。作業路徑本身不繞行（覆蓋要求）：作業路徑上的衝突以**抬高高度**解決（§9.2 的 $z_{lo}$ 已含障礙頂 + $b_v$），僅轉移／進入／返航路徑做水平繞行。

### 7.6 障礙繞行（轉移／進入／返航）

**圓形（論文 Eq.27 推廣）**：繞行半徑 $r_D=\max(R_T+b_h,\ R_{eff})$，安全點 $P_{D,1},P_{D,2}$ 為自起點到圓 $r_D$ 的切點，取離原路徑較近側，$Y_1=D(S_i\to S_{D,1})+D(S_{D,1}\to S_{End})$（$S_{D,1}$ 之航向為切線方向、半徑 $r_D$）。

**多邊形**：以 $\rho=\max(b_h,R_{eff})$ 做**圓角偏移**（Minkowski sum with disc），其邊界僅由直線與半徑 $\rho\ge R_{eff}$ 的圓弧構成 → 沿邊界飛行本身即滿足曲率限制，且淨空 $\ge b_h$。在「切線圖」（tangent graph：各圓角弧之間、起終狀態圓與弧之間的公切線）上做 Dijkstra（邊權＝時間，含風），輸出 Dubins 相容路徑。若直接用可視圖（頂點連線）再做 Dubins 接合，須在每頂點額外緩衝 $R_{eff}(\sec(\Delta/2)-1)$（$\Delta$ 為航向變化）以補償弧切角；本設計採切線圖免此問題。僅當 $z$ 於該處低於 $z_{top}+b_v$ 時障礙物才進入切線圖（2.5D）。

```
AVOID(S_a, S_b, K, z_profile_hint):
  D0 = dubins6(S_a, S_b, R_eff); if clear(D0, K): return D0
  K_act = {K_o in K : z_hint(near K_o) < z_top_o + b_v}
  G = tangent_graph({circle(S_a), circle(S_b)} ∪ round_offset(K_act, rho))
  path = dijkstra(G, weight = time_with_wind)
  return concat_dubins(path)   # 每段為圓弧或公切線，曲率 <= 1/R_eff
```

### 7.7 可飛性論證

(i) 所有曲線段半徑 $\ge R_{eff}\ge R_{theo}(1+W/V_a)^2$ → 任意相位需求傾角 $\le\phi_{\max}$；(ii) 直線段 $V_g$ 由風三角保證存在（$|W_\perp|<V_a$ 於 PRECHECK 檢查）；(iii) 高度剖面坡度以地面坡度 $\gamma^g(\theta)$ 限制 → 空氣坡度 $\le3^\circ/4^\circ$；(iv) 轉彎中同時爬升：空氣坡度不變（$\dot z=V_a\sin\gamma$），但傾角下的升力需求增加，故在弧段限制 $\gamma\le\gamma^{air}\cos\phi_{\max}$（Pioneer 2.82°，HERO 2.72°）——包絡計算對弧段樣本以此值代入；(v) 離散化偏差 $\le e_{\max}$；(vi) 速度落在 $[V_{a,\min},V_{a,c}]$。

---

## 8 異構 IDP（改良動態規劃）

沿用論文 Algorithm 2 之逐階段排列列舉，改動：(a) 成本為時間；(b) 狀態 $S_i(s)$ 含高度與能量分率；(c) 同一群組 $\mathcal{U}_{k,c}$ 僅含同構型（條帶集依構型分開；$req$=both 時 A、B 各跑一次 IDP）；(d) 多胞：群組負責的所有胞的條帶合併為一集合 $\mathcal{L}$，跨胞轉移由 §7.6 AVOID 計算；(e) 加入返航段與能量硬限制；(f) beam width $\beta_{bw}$ 控制列舉。

轉移代價（論文 Eq.33 → 時間、含完工平衡）：

$$W\big(S_i(s),L_{l'}\big)=\frac{|\overset{\leftrightarrow}{t}^{trans}_{i,l_{i,s},l'}+\overset{\leftrightarrow}{t}^{oper}_{i,l'}|}{T_i(s)}\cdot\big(1+\kappa_{bal}\tfrac{T_i(s)-\bar T(s)}{\bar T(s)}\big)$$

```
IDP_HET(group U_g (same type c), strip set L, S_init, params):
  s=1; T_i=0; f_i=f_use; R_i=[]; L_nad=L; S_cur = S_init
  build T-matrix (entry row from S_init; transfer rows by dubins6/AVOID; oper by §7.2) for each i
  while L_nad ≠ ∅:
     if |L_nad| > |U_g|:
        cand = permutations(L_nad, |U_g|) × directions (fixed by Eq.24–26 alternation)
        cand = prune_top(cand, beam=beta_bw, key=Σ_i W)                 # beam
        for each assignment a in cand:
            for each i: check_energy(f_i - Δf(a_i) - f_RTB(end(a_i)) >= 0.30)  else discard
            cost(a) = max_i (T_i + Δt(a_i))  + eta * Σ_i Δt(a_i)         # min-max 主、總和次
        a* = argmin cost
     else:
        cand = permutations(U_g, |L_nad|) ; same evaluation ; a* = argmin
     apply a*: update S_cur, T_i, f_i, R_i, L_nad
     s += 1
  for each i: append RETURN(S_cur_i → best alternate/home)  (§9.6) ; T_i += t_return
  return {R_i, T_i, f_i}
```

退化性質：$\kappa_{bal}=\eta=0$、$\beta_{bw}=\infty$、單構型無風 → 與論文 Eq.(33)(34) 等價（時間 = 距離/常數 $V$）。

---

## 9 三維高度與起降走廊（幾何核心）

### 9.1 地面坡度限制

沿航跡樣本 $j$ 的方位 $\theta_j$（弧段逐樣本變動）：

$$\tan\gamma_{c,j}^g=\frac{V_a\sin\gamma_c^{air}\,\kappa_{turn,j}}{V_g(\theta_j)},\quad\tan\gamma_{d,j}^g=\frac{V_a\sin\gamma_d^{air}}{V_g(\theta_j)},\quad \kappa_{turn,j}=\begin{cases}\cos\phi_{\max}&\text{弧段}\\1&\text{直線}\end{cases}$$

（保守：$V_g$ 取該方位之地速；若以 $V_a$ 代替則順風時會超過 3° 空氣坡度。）下降時保守取 $\tan\gamma^g_d$ 以 $V_g$ **最大**值（順風）計算——亦即逆風時實際可更陡，我們不利用。

### 9.2 高度包絡（slope-limited envelope）

沿整條架次 2D 航跡以步長 $d_d$（預設 25 m；弧段 $\le R\Delta\psi$）取樣 $j=0..N$，每樣本：

- 走廊地形：$z_g^{\max}(j)=\max$ over 橫向 $|v|\le b_j$（作業段 $b_j=w/2$，其他段 $b_j=b_{trk}$=50 m）；$z_g^{\min}(j)$ 同理取 min。
- 障礙：$z_{obs}(j)=\max\{z_{top,o}+b_v:\ (x_j,y_j)\in\mathcal{K}_o\}$。
- **下界** $z_{lo}(j)=\max\{z_g^{\max}(j)+h_{clr},\ z_g^{\max}(j)+h_{\min fw,c},\ z_{obs}(j)\}$（作業段）；轉移段可略去 $h_{\min fw}$ 以外的感測條件但仍 $\ge z_g^{\max}+h_{\min fw}$。
- **上界** $z_{hi}(j)=\min\{z_g^{\min}(j)+h_{\max,c},\ z_g(j)+h_{ceil},\ z_{airspace}(j)\}$（$h_{\max,c}$ 為感測器上限：GSD／測距；法規 $h_{ceil}$ 以正下方地面計）。
- **目標** $z_{des}(j)=z_g^{\max}(j)+h_{nom,c}$（作業段）；轉移段 $z_{des}=$ 前後作業段目標之線性內插。

最小可行剖面（閉式）：

$$z^*(j)=\max\Big\{\underbrace{\max_{j'\le j}\big[z_{lo}(j')-\textstyle\sum_{m=j'}^{j-1}d_d\tan\gamma^g_{d,m}\big]}_{\text{通過波峰後只能以 }4^\circ\text{ 下降}},\ \underbrace{\max_{j'\ge j}\big[z_{lo}(j')-\textstyle\sum_{m=j}^{j'-1}d_d\tan\gamma^g_{c,m}\big]}_{\text{抵達波峰前必須以 }3^\circ\text{ 提前爬升}}\Big\}$$

兩個 $O(N)$ 遞迴即得：

```
ENVELOPE(z_lo[0..N], z_hi[0..N], tan_c[], tan_d[], d, z_pin_start=None, z_pin_end=None):
  a[0] = z_lo[0] if z_pin_start is None else max(z_lo[0], z_pin_start)
  for j in 1..N:   a[j] = max(z_lo[j], a[j-1] - d*tan_d[j-1])        # 前向：下降角限制
  b[N] = z_lo[N] if z_pin_end is None else max(z_lo[N], z_pin_end)
  for j in N-1..0: b[j] = max(z_lo[j], b[j+1] - d*tan_c[j])          # 後向：爬升角限制
  zs = max(a, b)
  # 起點釘住（自轉換完成點／前一段末端）之上界：不能比從起點爬升更高
  if z_pin_start: u[0]=z_pin_start; for j: u[j] = min(z_hi[j], u[j-1] + d*tan_c[j-1]) ; else u = z_hi
  if any(zs[j] > u[j]): return INFEASIBLE(at j)      # 見 9.4 處置
  return zs, u
```

註：簡報 2.1 第 2 點將兩趟命名對調；數學上「前向遞迴必須用下降角、後向遞迴必須用爬升角」，否則得到的不是可行下包絡。取 $\max(a,b)$ 後 $z^*$ 是滿足 $z\ge z_{lo}$、$-\tan\gamma_d^g\le dz/ds\le\tan\gamma_c^g$ 的**最低**剖面（逐點下確界，且本身可行——因 $a$、$b$ 各自滿足一側坡度限制且相互不衝突：$a$ 只在下降側受限，$b$ 只在爬升側受限，二者最大值在任意點的左右斜率皆落在限制內）。

最終剖面選取：$z(j)=\mathrm{clip}(z_{des}(j),\ z^*(j),\ u(j))$ 後**再跑一次 ENVELOPE**（以 $z$ 為新的 $z_{lo}$）確保坡度可行；由於 $z\ge z^*$ 且 $z^*$ 可行，第二次結果 $\le u$ 除非 $z_{des}$ 本身太陡，此時 clip 回 $u$。

### 9.3 變動 AGL 對條帶寬度之回饋

$AGL(j)=z(j)-z_g^{\max}(j)$，條帶 $l$ 之 $h_{\min,l}=\min_{j\in l}AGL(j)$。§5.4 的 `local_slope_envelope` 只以該條帶（兩端各延伸 $R_{eff}$，端點不釘住）計算，得到局部最小剖面 $z^*_{loc}$；全架次剖面（§9.2 含轉移段釘住）滿足 $z\ge z^*_{loc}$（同樣的 $z_{lo}$、多了釘住只會更高），故 $AGL_{final}\ge h_{\min,l}$ → §5.6 的寬度保證成立。反向風險是 $AGL$ 過**高**超過 $h_{\max,c}$（GSD 變粗）：§9.2 上界 $u$ 檢出 → §9.4。

### 9.4 不可行處置（依序）

1. 若 $z^*>u$ 僅在轉移段：以 §7.4 延長轉移（螺旋降高）。
2. 若在作業段（山谷／稜線交替使 3° 追不上）：(a) 該胞改用沿等高線候選方向（$T_{proxy}$ 之 $\text{pen}_{slope}$ 已偏好）；(b) 以最陡處為界將胞再分割（沿等高線切）；(c) 對 B 允許局部 GSD 放寬 $k_{GSD}$（預設 1.2，記錄於品質報告）；(d) 報告不可覆蓋子區。
3. 若 $z_{lo}>z_{hi}$ 逐點（高度帶為空，例如法規 120 m AGL 與 Pioneer 最低 120 m AGL）：PRECHECK 已攔截；任務中出現則該胞標記為不可覆蓋。

### 9.5 起飛走廊

參數：$H_{ts}$（60/50）、$H_{tc}$（預設區間中值 90/80，airspeed 未驗證取上限 100/90）、垂直爬升率 $v_{z,up}$（預設 2.0 m/s）、轉換水平距離 $d_{tr}$（預設 A 400 m、B 300 m；來源 $d_{tr}\approx V_{tr}^2/(2a_{tr})+V_{tr}t_{hold}$，$a_{tr}=0.5$ m/s²，待實測）、逆風方位 $\theta_{tr}=\beta$（風來向；風 <2 m/s 時取障礙最少方位）。

```
TAKEOFF_CORRIDOR(site H, wind, K, target_state S_entry, z_entry):
  P0 = H ; z0 = z_g(H)
  # 1 垂直爬升
  seg1 = vertical(P0, z0 -> z0 + H_ts), t = H_ts / v_z_up, power = hover
  # 2 轉換（逆風）：地面直線 d_tr 沿 theta_tr，高度 H_ts -> H_tc（線性），檢查地面淨空
  P_tc = P0 + d_tr * t_hat(theta_tr); assert corridor_clear(P0->P_tc, half_width = r_site + b_obs, z_lo = z_obs + b_v)
  S_tc = (P_tc, theta_tr, R_eff)
  # 3 固定翼爬升到任務高度
  Δh = z_entry - (z0 + H_tc);  D_need = Δh / tan(gamma_c^g(theta_tr))
  D_path = |dubins6(S_tc -> S_entry)|
  if D_path * tan_g_avg >= Δh:  return [seg1, seg2, dubins(S_tc->S_entry) with ENVELOPE(pin z0+H_tc)]
  # 4 爬升盤旋（ground-referenced orbit, R_orb >= R_eff）
  C = choose_orbit_center(near P_tc, R_orb, K, prefer upwind side); T_lap = ∮ ds / V_g(theta)
  Δh_lap = V_a * sin(gamma_c^air) * cos(phi_max) * T_lap        # 弧段爬升修正
  n_lap = ceil((Δh - D_straight_after*tan_g) / Δh_lap)
  # 出口切點 ϑ*：最小化 t_in = t(orbit partial) + t(dubins(S_exit(ϑ) -> S_entry)) s.t. 高度足夠
  ϑ* = argmin_ϑ ...
  return [seg1, seg2, join(S_tc->orbit), orbit×n_lap, exit at ϑ*, dubins(S_exit->S_entry)]
```

數值：Pioneer $R_{orb}=200$ m、無風每圈 $1257$ m、爬 $2\pi R\sin3^\circ\cos20^\circ=61.8$ m；HERO $R=120$ m → $754\cdot\sin3^\circ\cos25^\circ=35.8$ m/圈。自 $H_{tc}$=90 至 200 m AGL 需約 2 圈（Pioneer），若直線距離 $<2.1$ km。盤旋碟形區（半徑 $R_{orb}+b_{trk}$）須無 keep-out、且在 C2 內；地面參考盤旋（非隨風漂移）由 autopilot loiter 提供，$T_{lap}$ 隨風變動已由積分反映。

### 9.6 降落走廊（home／備降點通用）

```
LANDING_CORRIDOR(S_last, z_last, site T, wind, K):
  theta_app = beta(wind at H_tc)                       # 逆風進場（朝風來向飛）
  P_tc = T - d_tr_land * t_hat(theta_app)              # 轉換起點在下風側
  z_tc = z_g(T) + H_tc ; S_tc = (P_tc, theta_app, R_eff)
  # 下降門 P_des：需水平距離 D_desc = (z_last - z_tc)/tan(gamma_d^g)
  D_path = |AVOID(S_last -> S_tc)|
  if D_path * tan_d_g >= z_last - z_tc:
        path2d = AVOID(S_last -> S_tc); z = ENVELOPE(..., pin_start=z_last, pin_end=z_tc) with descent target
  else:
        C = choose_orbit_center(near P_tc upwind or over site, R_orb, K)   # 下降盤旋 / overhead spiral
        Δh_lap_d = V_a sin(gamma_d^air) cos(phi_max) T_lap                 # Pioneer ≈ 82 m/圈
        n_lap = ceil((z_last - z_tc - D_path*tan_d_g)/Δh_lap_d)
        path2d = AVOID(S_last -> orbit) + orbit×n_lap + exit -> S_tc
  # 轉換段：P_tc -> T 沿 theta_app，H_tc -> H_ts，減速至懸停；檢查 corridor_clear
  # 垂直降落：H_ts -> 0 at v_z_down (預設 1.0 m/s)，最後 10 m 0.5 m/s
  # 復飛（missed approach）：任一點中止 → 沿 theta_app 以 gamma_c 直線爬升至 z_tc + 50 m → 回下降盤旋
  return segments, t_land, f_land
```

站點檢查（每個 home／alternate）：淨空圓 $r_{site,c}$ 內無任何障礙；$b_{obs,c}$ 內無高於 $H_{ts}$ 之障礙；轉換走廊（$P_{tc}\leftrightarrow T$，半寬 $r_{site}+b_{obs}$）內障礙頂 + $b_v$ 低於走廊高度線 $H_{ts}+(H_{tc}-H_{ts})\,x/d_{tr}$；下降盤旋碟形區無 keep-out；風向變化 $>45^\circ$ 時重算走廊（§10）。

### 9.7 全架次 3D 組裝、平滑與點刪減

```
BUILD_3D(sortie 2D segments with types, c):
  samples = resample(all segments, d_d)                       # 段型別標記：TO / IN / OPER / TRANS / RET / LD
  compute z_lo, z_hi, z_des per sample (§9.2, per type)
  z*, u = ENVELOPE(..., pin_start = z0 + H_tc, pin_end = z_g(site)+H_tc)
  z = clip(z_des, z*, u); z = ENVELOPE(z as z_lo, u)          # 二次保證
  # 平滑：論文 Eq.37/38 之投影梯度下降（每步投影回 [z*, u]）
  for it in 1..N_it:  z_j -= lr * ( c1 sign(z_j - z_des_j) + 2 c2 (2 z_j - z_{j-1} - z_{j+1}) ) ; z = clip(z, z*, u)
  # 點刪減：論文 Eq.39（3D 共線判準）+ 最大間距 L_max（預設 500 m）+ 弧段保留
  keep j if |cos∠(P_{j-1}P_j, P_jP_{j+1}) - 1| >= eps_col or spacing > L_max or j is arc/segment boundary
  verify: for each OPER strip l: min AGL >= h_min_l ; max AGL <= h_max_c * k_GSD ; slopes within limits
  return waypoints (x,y,z,V_a*, sensor_on, arc params)
```

---

## 10 動態重規劃（幾何觀點）

觸發事件與幾何處置：

| 事件 | 處置 |
|---|---|
| UAV 退出 | 依論文：以當前狀態重初始化，找出未覆蓋（含部分覆蓋）條帶 → 部分覆蓋的條帶自最後有效覆蓋點裁短（保留 $e_{ext}$）→ 重配置＋IDP（僅同構型可接手 lidar／rgb 條帶；any 條帶可換構型但需以新構型寬度**重鋪該殘餘子多邊形**） |
| 新區域 | 新區 DECOMPOSE → $J_{k,c}$ → 就近／能量充足者接手；正在作業的機先完成當前作業路徑（論文 Fig.15 策略） |
| 低電量／能量偏差 $>\delta_f$ | 重算 $f_{RTB}$ 場；若違反 (C4) 立即插入 RETURN 至最近備降點 |
| 失鏈 | 策略參數 `link_loss_policy ∈ {RTB, continue_next_wp, nearest_alt}`；預設 RTB，路徑為預先上傳之 AVOID(當前→home) |
| 風場更新 | 重算 $R_{eff}$、$V_g$、$\gamma^g$；若 $R_{eff}$ 增大 → 轉移路徑重生（作業路徑不變）；若起降風向變 $>45^\circ$ → 重算走廊 |
| 目標線索（cue-and-confirm） | A 回報目標點 $p_t$ → 生成小圓形／方形區（半徑 $r_{cue}$，req=rgb，$val$ 高）→ 指派最近 B（能量可行）以 AVOID 進入、以 $2R_{eff}$ 為邊長之短條帶或單圈盤旋確認 |

重規劃保留「已飛過部分」的幾何：以當前狀態 $S_i(t)$（位置、航跡方位、高度、能量）為新的 $S_i$，進入路徑重新由 Dubins 求得；仍在作業路徑上者以完成該路徑之末端狀態為 $S_i$。

---

## 11 防撞／去衝突

1. **高度分層**：A、B 的作業 AGL 帶 $[h_{\min fw,c},h_{\max,c}]$ 先由感測需求決定；若 $|h_{nom,A}-h_{nom,B}|\ge\Delta h_{sep}$（預設 50 m）且兩帶在該區不重疊 → 允許 A、B 同時在同一胞（req=both）或相鄰胞作業。否則採 2/3。因 AGL 隨地形變動，比較的是同一 $(x,y)$ 之 $z$：檢查 $\min_j|z_A(j)-z_B(j')|$ over $|P_A(j)-P_B(j')|<d_{sep}$。
2. **水平分離**：不同機的胞集合互斥；迴轉帶（胞外 $R_{eff}+b_{trk}$）與相鄰胞的迴轉帶若重疊 → 進入 3。
3. **時間分離（4D 檢查）**：所有航跡以時間參數化（含 $\pm\sigma_t$，$\sigma_t=k_t\cdot t$，$k_t$ 預設 0.05），對所有機對計算最近距離；衝突（$d<d_{sep}$=100 m 且 $|\Delta z|<\Delta h_{sep}$）時，能量裕度較大者於前一轉移插入延遲盤旋（1 圈 = $T_{lap}$）或 IDP 重排該階段。
4. **起降走廊獨占**：每站點走廊為互斥資源；佔用區間 $[t_{tr,start}-\delta, t_{clear}+\delta]$；同站起飛間隔 $\Delta t_{dep}$（預設 90 s）；降落優先於起飛；備降點被佔用時次近備降點。
5. **同構型同胞多機**（論文情境）：IDP 的方向交替與「不同機不從同一路徑進入」已避免同路徑對飛；相鄰條帶反向平行飛行間距 $s\ge105$ m $\ge$ 側向分離門檻 $d_{sep,lat}$（預設 60 m）；skip-$k$ 序列使同一時刻相鄰佔用條帶相距 $\ge ks$。

---

## 12 複雜度

| 步驟 | 複雜度 | 備註 |
|---|---|---|
| DP 簡化 | $O(n\log n)$ | $n$ 頂點 |
| HM 凸分割 + ACD 合併 | $O(n\log n)+O(m^2)$ | $m$ 胞數 |
| 條帶鋪設（含局部包絡） | $O(m\cdot|C|\cdot n_l\cdot N_s)$ | $N_s$ 每條帶樣本數；迭代 $\le3$ |
| Dubins 矩陣 | $O(n_l^2)$ 次 6-word 求解，每次 $O(1)$ + keep-out 檢查 $O(N_s|\mathcal{K}|)$ | 可用空間索引降至 $O(N_s\log|\mathcal{K}|)$ |
| AVOID 切線圖 | $O(V^2\log V)$，$V$=弧數 | 只對衝突的轉移呼叫 |
| IDP | 每階段 $A^{\lambda}_{n_{nad}}$，beam 後 $O(\beta_{bw}\lambda)$，階段數 $\lceil n_l/\lambda\rceil$ | 論文 0.06–0.4 s 量級；本案 $n_l$ 較多（足跡窄）故 beam 必要 |
| ENVELOPE / 平滑 / 刪減 | $O(N)$ / $O(N_{it}N)$ / $O(N)$ | $N$ 全架次樣本數 |
| 4D 去衝突 | $O(n_U^2 N)$ | 空間網格加速 |

單區 $n_l\le 60$、$\lambda\le3$、$\beta_{bw}=200$ 時全流程目標 $<2$ s（Python+numpy），重規劃 $<1$ s（只重算受影響群組）。

---

## 13 與論文對照

| 論文元素 | 本設計 | 退化條件 |
|---|---|---|
| Eq.1 定速運動學 | §3.1 風三角、$V_a^*$、$R_{eff}$ | $W=0$、$V_a^*=V$ |
| Eq.2/3 梯形 FOV | §3.2 LiDAR／相機足跡（保守 AGL）；梯形為選項 | `fov_model=trapezoid`、常數 $h$ |
| Eq.4 DEM 雙線性 | 沿用；走廊取 max/min | — |
| DP 簡化+整體凸化 | §5.2 DP（只外擴）+ HM 凸分割 + ACD 合併 | 區本身凸 → 單胞 |
| $\alpha_k$ 最小寬度方向 | §5.3 候選集 + $T_{proxy}$（風、轉場、坡度） | $W=0$、平地 → 最小寬度方向 |
| Eq.5–9 條帶與端點 | §5.4 序列鋪設、變寬、$e_{ext}$、裁剪極值 | 常數 $w$ → Eq.6–9 |
| Eq.10–12 二元距離 | §7.2 二元時間 | 除以 $V$ |
| Eq.13–26 矩陣與限制 | 原樣沿用（元素改時間，$\infty$ 表不可行） | — |
| Eq.27 威脅繞行 | §7.6 圓形推廣 $r_D=\max(R_T+b,R_{eff})$ + 多邊形圓角偏移切線圖 | 僅圓形威脅 → Eq.27 |
| Eq.28/29 弧離散 | §7.5 以 $e_{\max}$ 定角步；弧命令選項 | $\Delta\psi=\psi_{\max}$ |
| Eq.30/31 $J_k,\lambda_k$ | §6 加 $\phi$ 適配度與硬限制 | $\omega_5=0$ |
| Eq.32–34 IDP | §8 時間、能量、返航、beam、平衡項 | $\kappa_{bal}=\eta=0$、beam ∞ |
| Eq.35 曲率補償 | ECEF→ENU 免補償 | 平面近似時啟用 Eq.35 |
| Eq.36 高度下降 | §9.2 雙向坡度包絡（閉式、最低可行剖面） | Eq.36 是其單向貪婪近似 |
| Eq.37–39 平滑／刪減 | §9.7 投影 GD（不穿包絡）、3D 共線 + $L_{\max}$ | 同 |
| 起降 | §9.5/9.6 新增 | 略去 → 論文起始狀態 |
| 動態事件 | §10 新增能量／失鏈／風／cue-and-confirm | 僅退出／新區 → 論文 |
| 防撞 | §11 新增 | — |

---

## 14 參數表（含表 1 全部鍵；值不得更改，來源標示 T1=表 1，D=本設計預設，M=任務輸入）

| 鍵 | Pioneer PNP (A) | HERO PNP (B) | 來源／用途 |
|---|---|---|---|
| platform_type | lift_and_cruise_vtol_4plus1 | 同 | T1；禁止懸停轉向 |
| wingspan_m | 3.20 | 2.18 | T1；起降淨空校核 $r_{site}\ge$ 翼展/2 + 裕度 |
| fuselage_length_m | 1.68 | 1.14 | T1；地面部署 |
| wing_area_m2 | 1.00 | 0.53 | T1；翼載 $AUW g/S$ 用於失速裕度提示 |
| mtow_kg | 24.9 | 7.0 | T1；AUW 上限硬檢查 |
| reference_auw_kg | 24.6 | 6.65 | T1；能量 AUW 修正基準 |
| reference_payload_kg | 5.0 | 0.6 | T1 |
| payload_max_kg | 5.0 | <1.0 | T1；酬載檢查 |
| cruise_airspeed_mps | 21.5 | 19.0 | T1；$V_{a,c}$ |
| cruise_airspeed_kmh | 77.4 | 68.4 | T1；顯示用 |
| min_mission_airspeed_mps | 18.0 | 15.0–16.0（規劃取 16.0） | T1；$V_{a,\min}$ |
| transition_airspeed_mps | 16.0 | 14.0 | T1；$V_{tr}$ |
| stall_airspeed_mps | 14.0 | 12.0 | T1；僅檢查 $V_{a,\min}>1.2V_s$ |
| max_airspeed_mps | 35.0 | 待實機確認（規劃不使用） | T1 |
| max_climb_angle_deg | 3.5 | 3.5 | T1；不用於規劃 |
| planning_climb_angle_deg | 3.0 | 3.0 | T1；$\gamma_c^{air}$ |
| max_descent_angle_deg | 5.0 | 5.0 | T1；不用於規劃 |
| planning_descent_angle_deg | 4.0 | 4.0 | T1；$\gamma_d^{air}$ |
| max_fw_climb_rate_mps | 1.13 | 1.00 | T1；時間換算校核 |
| max_fw_descent_rate_mps | 1.50 | 1.33 | T1；同上 |
| max_bank_angle_deg | 20 | 25 | T1；$R_{theo}$ 驗算、弧段爬升修正 |
| turn_radius_theoretical_m | 130 | 75 | T1；$R_{eff}$ 風放大基準 |
| turn_radius_planning_m | 200 | 120 | T1；$R_{plan}$ |
| vtol_site_clear_radius_m | 8 | 6 | T1（暫定）；站點檢查 |
| obstacle_horizontal_buffer_m | 30 | 25 | T1（暫定）；站點檢查 |
| transition_start_agl_m | 60 | 50 | T1（placeholder）；$H_{ts}$ |
| transition_complete_agl_m | 80–100（D 取 90；未驗證取 100） | 70–90（D 取 80；未驗證取 90） | T1（placeholder）；$H_{tc}$ |
| min_fw_operation_agl_m | 120 | 100 | T1；$h_{\min fw}$ |
| wind_resistance_public | Beaufort 5 | Beaufort 5 | T1；非放行門檻 |
| reference_endurance_min | 176 | 136 | T1；$t_{ref}$ |
| reference_total_range_km | 242 | 144 | T1；距離驗算 |
| reserve_fraction | 0.30 | 0.30 | T1；硬限制 |
| usable_range_fraction | 0.55–0.60 | 0.55–0.60 | T1；驗算 |
| usable_distance_km | 133–145 | 79–86 | T1；驗算 |
| symmetric_mission_radius_km | 66–72 | 39–43 | T1；$r_{sym}$（半徑硬限制取下限） |
| airspeed_sensor_validated | false | false | T1；轉換保守化 |
| battery_monitor_validated | false | false | T1；$k_{bm}=0.9$ |
| c2_link_range_km | 待填入（M 必填） | 待填入（M 必填） | T1；缺則拒絕規劃 |
| terrain_clearance_m | 待任務定義（M 必填；建議 ≥ DEM 誤差×3 + 30） | 同 | T1；$h_{clr}$ |
| alternate_landing_sites_required | true | true | T1；缺備降點拒絕 |
| **新增（D）** | | | |
| lidar_usable_scan_deg / pulse_rate_hz / point_density_req | 70 / M / M | — | 足跡、$V_g^{cap}$ |
| cam_focal_mm / sensor_w_mm / sensor_h_mm / pixel_um / shutter_hz / gsd_req_m | — | M | 足跡、$h_{\max}$、$V_g^{cap}$ |
| side_overlap / forward_overlap | 0.25 / — | 0.30 / 0.75 | 間距、地速上限 |
| settle_time_s | 2.0 | 2.0 | $e_{ext}$ |
| wind_go_mps / gust_go_mps / crosswind_site_max_mps | 8 / 12 / 6 | 8 / 12 / 5 | go/no-go |
| v_z_up_mps / v_z_down_mps | 2.0 / 1.0 | 2.0 / 1.0 | 垂直段 |
| transition_ground_dist_m | 400 | 300 | 待實測 |
| hover_power_ratio k_h / climb_power_factor k_climb | 4.0 / 0.35 | 4.0 / 0.35 | 待實測 |
| e_max_discretize_m / psi_max_deg | 3 / 30 | 3 / 30 | Dubins 離散 |
| track_half_width_b_trk_m / delta_edge_m | 50 / 15 | 50 / 15 | 走廊、縫隙緩衝 |
| obstacle_vertical_buffer_b_v_m | 30 | 30 | keep-out |
| h_ceil_agl_m | M（法規；預設 120 → Pioneer 需豁免） | M | 上界 |
| dh_sep_m / d_sep_m / d_sep_lat_m | 50 / 100 / 60 | 同 | 去衝突 |
| dt_dep_s / k_t | 90 / 0.05 | 同 | 走廊互斥、時間不確定度 |
| acd_tau_factor / dp_eps_factor | 0.5 / 0.1 | 同 | 分解 |
| beam_width / kappa_bal / eta | 200 / 0.3 / 0.05 | 同 | IDP |
| sample_step_d_d_m / smooth_c1 / smooth_c2 / N_it / eps_col / L_max_m | 25 / 1.0 / 0.5 / 50 / 1e-3 / 500 | 同 | 3D |
| k_GSD_relax | — | 1.2 | §9.4 |
| link_loss_policy | RTB | RTB | §10 |
| fov_model | lidar | camera（可選 trapezoid） | §3.2 |

---

## 附錄 A：主流程

```
HV_DCCPP(inputs):
  params = PRECHECK(inputs)
  for Z_k: cells[k] = DECOMPOSE(Z_k, req_k, params)            # §5（含各構型條帶集、殘帶）
  assign = ALLOCATE(cells, fleet, params)                       # §6（J_kc, λ_kc, 硬限制）
  for each group g = (U_g, cell list, type c):
      S_init_i = TAKEOFF_CORRIDOR(H_i, ...) exit state, ∀ i ∈ U_g        # §9.5
      plan_g = IDP_HET(U_g, strips(cells of g), S_init, params)          # §8（含 RETURN）
      for i in U_g:
          sortie2d = takeoff_i + entry + Σ(oper, trans) + return + LANDING_CORRIDOR(...)   # §9.6
          wp_i = BUILD_3D(sortie2d, c)                                    # §9.7
          verify coverage (§5.6), flyability (§7.7), energy (§3.4), link (§4)
  DECONFLICT(all wp)                                                      # §11
  return mission plans, coverage report, quality report (GSD/密度統計, 不可覆蓋子區)
```

## 附錄 B：關鍵數值範例（無風／8 m/s）

| 量 | Pioneer | HERO |
|---|---|---|
| $R_{eff}$（W=0 / 8 m/s） | 200 / 245 m | 120 / 152 m |
| $w$ @ 最低作業 AGL | 168 m（120 m, 70°） | 150 m（100 m, 36 mm 幅寬/24 mm 焦距） |
| $s$（側重疊 0.25 / 0.30） | 126 m | 105 m |
| skip-$k$（W=0） | $\lceil400/126\rceil=4$ | $\lceil240/105\rceil=3$ |
| Ω 轉相鄰（W=0） | $R(\pi+4\arccos(0.658))$=1.31 km | $R(\pi+4\arccos(0.719))$=0.75 km |
| U 型 skip-$k$ | $\pi R+(ks-2R)$=732 m | 452 m |
| 每圈爬升／下降（弧段修正） | 61.8 / 82.4 m | 35.8 / 47.7 m |
| 3° 地面坡度 W=8 順風 | 2.18° | 2.11° |
| $H_{tc}\to$150 m AGL 直線需距 | 1.15 km（W=0） | 1.34 km |
