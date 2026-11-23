# ADR-0007: 以 hv_dccpp 作為異構雙構型 VTOL 的覆蓋規劃核心

> ⚠ 本文件寫於改名之前。文中的 **HV-DCCPP** 就是今日的 **HARVEST**（見 [ADR-0010](0010-harvest-naming.md)）。ADR 是有日期的紀錄，內文刻意不改寫。


**狀態**: Accepted
**日期**: 2026-09-06
**決策者**: @james zheng

## Context / 背景

`core/dccpp/` 是 Wang et al. 2025（*Defence Technology* 47:197-215）的忠實實作，
但它把「一支機隊」當成同質的：

* **逐機刈幅表達不出來**。`fov_width` 在 `mission/swarm_coordinator.py:1090` 被寫進
  `UAVState`，全樹沒有任何地方讀它；區域分解只吃單一 `coverage_width_m`。
* **全隊共用一個轉彎半徑與一個高度**。`swarm_coordinator.py:1184` 取
  `max(u.turn_radius ...)`，UI 也只有一組 spinbox。用 200 m 規劃 HERO 過度保守，
  用 120 m 規劃 Pioneer 則直接飛不出來。
* **沒有能量、沒有風、沒有起降走廊**。`config/vehicle_profiles.yaml` 有
  `max_flight_time_min` 但無人讀；`AltitudePlanner` 存在，但 live 路徑
  （`swarm_coordinator.py:1177-1298`）從來沒呼叫過它。
* **`'vtol'` 被靜默降級成 `MULTIROTOR`**（`swarm_coordinator.py:1080-1082`；
  `VehicleType` 根本沒有 VTOL 成員），於是 QuadPlane 用多旋翼的 haversine 估成本。

已有一套完成的函式庫 `hv_dccpp`（同一篇論文，24 模組、809 項測試）把這些補齊，
並且針對兩台真實機（Pioneer PNP + 5 kg LiDAR / HERO PNP + 0.6 kg 相機）建模。
其轉接層 `hv_dccpp.integration.aeroplan_adapter.HvDccppCoordinator.plan_coverage_dccpp`
的簽章與回傳的 6 鍵字典與 `mission/swarm_coordinator.py:975` **逐字相同**。

## Decision / 決策

在 `SwarmCoordinator.plan_coverage_dccpp` **內部**加一個旗標分支
（`self.use_hv_dccpp`，預設 `False`），插在 DEM 轉成 `terrain_func` 之後、
建立 `UAVState` 之前，成立時直接回傳 hv 的 6 鍵字典。

* 下游（NFZ 重繞、避撞、地圖、統計、`.waypoints` 匯出、SITL 上傳、spawn home 推導）
  **一行都不用改**，因為回傳結構與 duck typing 欄位完全同形。
* UI 閘門：`vehicle_type == 'vtol'` **且** params 帶 `hv_dccpp=True`。
  入口只有一個 —— 「VTOL 偵察」分頁「5. 規劃」的**規劃演算法**下拉
  （`DCCPP（既有實作）` / `HV-DCCPP 異構雙構型（本專案）`）。
  一般 DCCPP 分頁刻意不放這個開關：同一個旗標兩個入口，操作員會不知道自己按的是哪一套。
* 構型 A/B 由 `hv_class_of` 回呼決定（面板的「構型 A 台數」）。**這個回呼不能省**：
  UI 組出來的 drone dict 沒有任何構型線索，轉接層的預設規則會把整隊當成 Pioneer，
  結果完全錯誤而且不報錯。
* `hv_dccpp` 沒安裝時 `ImportError` 被接住，記一行 warning 後降級回既有 DCCPP。

## Alternatives Considered / 考慮過的替代方案

1. **另開一個 UI 分頁與 signal，兩套規劃器並存（手冊 §6「方式 B」）**。
   好處是完全不碰 `swarm_coordinator.py`，可做 A/B 比較；壞處是所有下游
   （匯出、上傳、SITL spawn、動態重規劃）都要複製一份 handler，而它們原本就
   只讀 `self._dccpp_result`。既然回傳同形，複製沒有換到任何東西。
2. **把 hv 的平台常數搬進 `config/vehicle_profiles.yaml`**。該結構表達不了
   `PlatformSpec` 的 40+ 欄位，而且數值一旦分叉，`docs/平台參數表.md` 就不再是
   單一權威來源。改為只在 YAML 放指標性條目（給 UI 顯示），數值一律由 hv 提供。
3. **直接改造 `core/dccpp/` 支援異構**。等於把 hv_dccpp 重寫一次，且會動到
   `tests/test_dccpp_pipeline.py` 那組 golden。投報率不成立。

## Consequences / 後果

### Positive
- 逐機構型：轉彎半徑（含風修正 `r_eff`）、巡航空速、爬降角、續航、最低作業高度。
- 刈幅由感測器與作業高度推導（160 m AGL 實測 A 224 m、B 235 m），而不是一個
  寫了沒人讀的欄位。
- 能量閉合檢查（含 30% 保留）、風、坡度受限高度包絡、垂直起降走廊、4-D 去衝突。
- 旗標預設關閉 ⇒ `tests/test_dccpp_pipeline.py` 的 14 個 golden 值不受影響。

### Negative
- **`total_makespan` 的語意變了**：hv 含起降、MC↔FW 轉換、盤旋等高與 D46 錯開起飛，
  必然大於既有的「路徑總長 ÷ 速度」。A/B 比較要比 `total_distances` 或
  各機 `total_length_m`，不要比 makespan。
- 只支援 lift-and-cruise 4+1 VTOL 兩個構型；多旋翼與純固定翼不可走這條分支。
- 座標差約 0.1%（hv 全程 WGS84 橢球，AeroPlan 部分模組用 111111.0 m/deg）
  ⇒ 既有 golden 不能拿來驗收 hv 分支，另建 `tests/test_hv_dccpp_adapter.py`。
- hv 分支下 UI 的 25 鍵只有 4 個真的生效（`overlap_rate`、`altitude`、
  `min_separation_m`、`alt_offset_m`），其餘被平台常數或「hv 一律做」取代。
- 山區 8 機情境仍有 4 筆殘餘 4-D 衝突（hv 端的已知限制）。

### Neutral
- 既有的 NFZ 重繞與避撞後處理在 hv 分支下是多餘的（hv 已避開 NFZ、已 4-D 分離），
  建議在 UI 取消勾選避撞，否則會在已分離的路徑上再疊一次高度偏移。
- 參數逐鍵去向、錯誤訊息對照、SITL 驗收清單見
  `C:/Users/joy46/Documents/Projects/vtol演篹法/docs/AEROPLAN_接入手冊.md`。
