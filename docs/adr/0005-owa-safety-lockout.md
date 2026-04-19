# ADR-0005: OWA-UAV 禁 RTL 參數的安全機制

**狀態**: Accepted
**日期**: 2026-04

## Context

`mission/owa_uav_params.py` 為模擬 Shahed-136 類 One-Way Attack UAV 生成
ArduPlane .parm 檔，**刻意禁用 failsafe + 禁用 RTL**：

```
FS_LONG_ACTN = 0   (訊號中斷 → Continue in AUTO)
FS_SHORT_ACTN = 0  (短時失效 → Continue)
RTL_AUTOLAND = 0   (禁用自動降落)
MIS_DONE_BEHAVE = 0 (任務完成 HOLD，不回家)
```

這是戰術需求（不可召回的自殺攻擊機），但**若誤上傳到實機會造成巨大安全事故**：
- 訊號中斷時無人機持續朝預設座標飛
- 無法以遙控器召回
- 可能傷及無辜

## Decision

**雙層防護機制**：

### 1. 產出檔案第一行警告標頭
```parm
# !!! WARNING / 警告 !!!
# 本參數檔 "禁用 failsafe + 禁用 RTL"，僅供 SITL 模擬驗證用。
# 嚴禁直接上傳至實機飛控 (ArduPlane) — 會導致訊號中斷時
# 無人機「繼續朝預設座標飛行且無法召回」，造成重大安全事故。
# 實機部署請使用 plane.parm 預設值 + FS_LONG_ACTN=1(RTL)。
```

### 2. 執行階段 `logger.warning` 提示
```python
logger.warning(
    '[OWA] 產生 SITL 戰術參數檔 (禁 RTL / 禁 failsafe)，'
    '請確認僅用於模擬，切勿上傳實機！'
)
```

## Alternatives Considered

### A. 不提供此功能
- ❌ 使用者有合法研究需求（SITL 驗證 OWA 行為）
- ❌ 可能用其他方式繞過（直接編輯 .parm）

### B. 強制檢測主機是否為 SITL 才輸出
- ⚠️ 難以 100% 檢測（SITL 與實機 MAVLink 相同）
- ⚠️ 會阻擋合法測試場景（例如 HIL）

### C. 線上簽章驗證
- ❌ 過度設計，實用性低

## Consequences

### Positive
- 使用者在使用瞬間看到警告（很難視而不見）
- 檔案本身自帶警告，即使轉存到他人電腦仍可見
- 符合軍工軟體的「預設安全」原則

### Negative
- 真的想誤用的人仍可刪除警告 → 這是技術無法解決的問題
- 警告在 log 中可能被忽略

### Neutral
- 未來若要再強化：可加 runtime SITL 偵測（如檢查 SYSID 範圍）
