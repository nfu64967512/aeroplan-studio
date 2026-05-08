# ADR-0004: STOT / DTOT 為何有兩組不同定義？

**狀態**: Superseded by the `SAME`/`DIFF` rename (見下方 2026-04 更新)
**日期**: 2026-04

## Context

早期版本定義：
- **STOT** = Simultaneous Launch / Time On Target 同地發射、同時命中
- **DTOT** = Different Launch / Time On Target 異地發射、同時命中

兩者的「TOT」都指同時命中，差異在**發射位置**。

後來戰術需求擴增「**波次打擊**」：多機在 T, T+Δ, T+2Δ 依序命中。
這時再用舊的 STOT/DTOT 語彙就撞名了。

## Decision

**把「發射位置」與「時間協同」拆成兩個正交概念**：

| 維度 | 值 | 說明 |
|---|---|---|
| 發射位置 | `DTOT`  (異地) | 各機自動分散於目標周圍 |
|  | `STOT`  (同地) | 所有機共用一個發射基地 |
| 時間協同 | `STOT` (同時) | 全體同一秒命中 |
|  | `DTOT` (間隔) | slot k 於 T + k·Δ 命中（波次） |

UI 分成兩個下拉選單：
- 頂部「發射位置」：控制 `plan_auto()` vs `plan_stot()`
- 底部「時間協同」：控制 `DTOTCoordinator.coordinate(mode, interval_sec)`

這樣可以自由組合出 4 種戰術：
- 異地+同時：各據點同秒全向打擊（典型飽和）
- 異地+間隔：各據點波次打擊（持續壓制）
- 同地+同時：一個基地爆發出擊
- 同地+間隔：一個基地波次連發

## Alternatives Considered

### A. 重新命名，避免撞名
- 考慮過 `TIME_SIM` / `TIME_STAG` 等，但軍事領域 STOT/DTOT 有歷史意義
- ❌ 違反領域慣用語

### B. 合併成單一「戰術模式」，6 種 enum
- ❌ 參數爆炸，UI 複雜

### C. 完全捨棄舊定義
- ❌ 破壞舊使用者理解

## Consequences

### Positive
- 4 種戰術組合自然衍生，無需新 enum
- 保留軍事術語

### Negative
- 使用者需理解「兩個 DTOT 是不同的事」，UI tooltip 必須清楚說明
- `_on_strike_execute` 同時處理兩個 mode 參數，需小心不要混用

### Neutral
- ADR 本身就是為了記錄這個語彙衝突；未來人遇到疑問可查此文件


---

## 2026-04 更新：徹底消除歧義 — 引入 `SAME`/`DIFF`

### 新決策

保留舊決策中「兩個正交維度」的思想，但**改名內部值以消除撞字**：

| 維度 | 舊值 | 新值 | UI 顯示 |
|---|---|---|---|
| 發射位置 (`launch_mode`) | `STOT` / `DTOT` | **`SAME` / `DIFF`** | 「同地發射」/ 「異地發射」 |
| 時間協同 (`timing_mode`) | `STOT` / `DTOT` | `STOT` / `DTOT`（保留） | 「同時命中」/ 「間隔命中」 |

現在 `STOT` 只有一個意義（同時命中），`DTOT` 只有一個意義（間隔命中）。

### 向下相容

新增 `core.strike.time_coordination.normalize_launch_mode()`：

```python
normalize_launch_mode('STOT')  # → 'SAME'  (舊值自動轉換)
normalize_launch_mode('DTOT')  # → 'DIFF'
normalize_launch_mode('SAME')  # → 'SAME'  (no-op)
normalize_launch_mode('同地')  # → 'SAME'  (中文也接受)
```

所有接收 `mode` 參數的地方在內部先 normalize 再分流，舊資料 (.aeroplan 專案檔、
舊 log) 皆可讀。

### 影響範圍（已全部應用）

- `parameter_panel.get_strike_launch_mode()` 回傳 `SAME`/`DIFF`
- `strike_controller._on_strike_execute()` 用 `SAME`/`DIFF` 做 `plan_stot`/`plan_auto` 分流
- `strike_controller._on_strike_execute_vtol()` 同樣用 `SAME`/`DIFF`
- `strike_controller._on_strike_mode_changed()` 接受兩組值 (透過 normalize)
- `config.schemas.StrikeParameters.launch_mode` 預設值改為 `DIFF`，接受 4 種值
