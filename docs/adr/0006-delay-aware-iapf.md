# ADR-0006: 延遲感知改進型 APF (IAPF) 設計

**狀態**: Accepted
**日期**: 2026-04

## Context

傳統人工勢場法 (APF) 避障有兩大經典問題：

### 1. GNRON (Goal Non-Reachable due to Obstacles Near)
當 UAV 靠近目標時，附近障礙（鄰機）的斥力可能超過引力，
導致 UAV 「被推離目標」無法到達。

### 2. 局部極小值 (Local Minima)
當引力與斥力方向剛好相反時合力為零，UAV 卡住不動。

### 3. 固定翼物理限制
APF 輸出的是加速度向量，對四旋翼可直接用，但對固定翼：
- 不能原地懸停（V > V_stall 恆成立）
- 不能直角轉彎（受 R_min 限制）
- 不能原地煞停

### 4. 通訊延遲
多機協同時鄰機位置訊息有 0.1-2 秒延遲，若直接用過期位置算斥力，
可能已經撞上。

## Decision

採用 **Ge & Cui 2000 改進公式 + 運動學夾限 + 延遲外推**：

### ① 斥力公式 (解決 GNRON)
```
F_rep_1 = k·(1/ρ − 1/ρ₀) · (ρ_goal^n / ρ²) · r̂     ← n=2 衰減
F_rep_2 = (n/2)·k·(1/ρ − 1/ρ₀)² · ρ_goal^(n−1) · ĝ  ← 拉向目標
η = exp(−λ·ρ₀/ρ_goal)                                ← 指數封套
F_rep = (F_rep_1 + F_rep_2) · η
```

當 UAV 靠近目標 (ρ_goal → 0)，`ρ_goal^n → 0`，斥力自動消失。

### ② 虛擬目標逃逸 (解決局部極小)
偵測 `|F_total| < ε_dead` 且 `距離目標 > ρ_arrived` ⇒ 死鎖：
```
q_virtual = q_self + R_look · [cos(ψ − π/2), sin(ψ − π/2), 0]
```
沿當前航向右側 90° 偏移（符合 ICAO 右手避讓慣例）。

### ③ 運動學夾限 (固定翼適配)
```
ψ̇_max = V / R_min
Δψ_clamp = sign(Δψ) · min(|Δψ|, ψ̇_max · Δt)
|v_z| ≤ V · sin(γ_max)   (γ_max ≈ 8°)
```

### ④ 延遲補償
```
t_delay = t_now − neighbor.last_update_time
p_now = p_rx + v_rx · t_delay
σ_pos = |v_rx| · t_delay · α     (α ≈ 0.3)
ρ_safe_effective = ρ_safe + σ_pos
```
有效安全距離膨脹，確保即使鄰機已轉向也不撞。

## Alternatives Considered

### A. 簡單 APF (不改進)
- ❌ GNRON / Local Minima 頻發
- ❌ 固定翼會輸出不可執行指令

### B. 切換到 RL (Reinforcement Learning)
- ⚠️ 訓練資料需求大、可解釋性差
- ⚠️ 難符合軍工驗證標準
- 📌 未來可考慮混合架構 (IAPF 作為 baseline policy)

### C. MPC (Model Predictive Control)
- ⚠️ 計算量大，多機即時規劃有挑戰
- ⚠️ 需要精確動力學模型

## Consequences

### Positive
- GNRON 從定義消失（數學保證）
- 實測 3 機密集包圍（距離目標 800m）能正確匯聚
- 延遲 0.5s 下仍有 ρ_safe_eff ≈ 803m 的安全餘裕

### Negative
- 虛擬目標的方向選擇目前 hardcode 右手；複雜場景可能需要左右試 both
- n=2、λ=1.5 等超參數需手調；無通用最佳值

### Neutral
- 實作於 `core/strike/advanced_recon_to_strike_manager.py`，
  有 standalone demo 可視覺化驗證
