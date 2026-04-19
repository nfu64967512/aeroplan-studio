# ADR-0001: 為何有 6 個 Strike Planner？

**狀態**: Accepted
**日期**: 2026-04

## Context

開發過程中陸續出現 6 個 Strike 相關 planner：

```
core/strike/
├── terminal_strike_planner.py       (基礎) Grid/Spiral + 三段式軌跡
├── swarm_strike_planner.py           1 目標、360° 包圍（獨立示範）
├── advanced_swarm_strike_planner.py  + STOT/DTOT-interval（獨立示範）
├── vtol_swarm_strike_planner.py      + VTOL + Phase 2/3 + CEP（VTOL 分流用）
├── dtot_coordinator.py               時間協同（與 terminal 搭配）
├── recon_to_strike_manager.py        DCCPP → Strike 動態切換
└── advanced_recon_to_strike_manager.py  + IAPF + 延遲補償（獨立示範）
```

這是因為需求演進：一開始只有基本 360° 包圍，之後陸續加入 STOT/DTOT、VTOL、
動態切換、延遲補償。每個需求增量時為了不破壞既有流程，選擇新增檔案。

## Decision

**保留 6 個 planner 並明確分類**：

| 類別 | 檔案 | 是否整合到主 UI |
|---|---|---|
| **Production (整合到 UI)** | terminal, dtot_coordinator, vtol_swarm, recon_to_strike | ✓ |
| **Standalone Demo (獨立示範)** | swarm, advanced_swarm, advanced_recon | ✗ |

Production 版：被 main_window 透過 StrikeControllerMixin 使用。
Standalone 版：可以 `python -m core.strike.X` 獨立執行 demo，作為學習/研究 reference。

**共用邏輯集中到 `core.strike.geometry`**（已於 2026 Q1 完成重構）。

## Alternatives Considered

### A. 合併成單一 `UnifiedStrikePlanner` 類
- ❌ 單檔會破 3000 行、條件邏輯爆炸（if vtol: ... if stot: ...）
- ❌ 增加新功能時每次都要碰大檔

### B. 全部重寫為策略模式 (Strategy Pattern)
- ❌ 需要重構 6 個檔案 + 所有呼叫點
- ❌ 高風險，報酬有限

### C. 用繼承層次 (BaseStrikePlanner + 5 個 subclass)
- ⚠️ 可行但需要統一資料結構 (StrikeTrajectory / VTOLPlan / etc.)
- 📌 留作未來重構方向

## Consequences

### Positive
- Standalone demo 可獨立執行，不需啟 UI
- 新需求（如 Swarm-of-Swarms）可以新建 planner，不污染既有邏輯
- 共用 `geometry.py` 已消除 80% 的重複程式碼

### Negative
- 新人上手需要先理解這個分類
- 6 個 planner 仍有一些概念重複（Target/UAV 資料類）

### Neutral
- 若哪天需要統一資料模型，方向 C 仍開放
