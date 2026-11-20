# ADR-0011: 作業高度以民航法規的 120 m AGL 為上限

**狀態**: Accepted
**日期**: 2026-09-07
**決策者**: @james zheng

> ⚠ **2026-09-08 訂正**：本文「Pioneer 在 120 m 上限下結構上飛不了」的結論，
> 成立的前提是沿用表 1 的 `min_fw_operation_agl_m = 120`。但表 1 那一列的使用方式欄
> 自己就寫「任務前需依地形、感測器與法規調整」，它是規劃值不是機體極限。
> 見 [ADR-0012](0012-min-fw-altitude-is-adjustable.md)：A/B 都設 100 m、
> 高度帶下限設 15 之後，法規 120 m 上限下兩型都有工作（makespan 1424 s）。

## Context / 背景

民航局把無人機的作業高度上限訂在 **120 m AGL**。此前「VTOL 偵查」面板把
`hv_waiver_max_agl_m` 預設成 **200**，等於替使用者假設他已經拿到 200 m 豁免；
規劃出來的任務違規，而畫面上完全看不出來。

改成法規值之後撞到兩件互相獨立的事。

### 一、AGL 上限（h_ceil）決定「哪個構型可以被指派」

上游 `Airspace.max_agl_m` 寫死 **120**（就是法規值），而

```
h_ceil = waiver_max_agl_m if given else max_agl_m          # platform.py:397
Δh_band = (min z_g + h_ceil) − (max z_g + max(h_minfw, h_clr))
可指派條件：Δh_band ≥ cfg.dh_band_min_m                      # 預設 20.0 m
```

平台參數表：Pioneer 的最低固定翼作業高度 `h_minfw` = **120 m**、HERO = 100 m。
所以在法規上限 120 m 之下：

- Pioneer：`120 − 120 = 0 m` ⇒ **永遠不可指派**。
- HERO：`120 − 100 = 20 m`，剛好等於下限，地形起伏一減就不足 ⇒ 也不可指派。

實測（2.45 km² 單區、作業高度 120 m、GSD 0.05、風 0）：

| AGL 上限 | 1A+1B | 2B | 1B |
|---|---|---|---|
| 120 m（法規值，無豁免） | **NO_GO** | **NO_GO** | **NO_GO** |
| 125 m | `['B']` 2085 s | `['B']` 1540 s | `['B']` 2130 s |
| 141 m | `['A','B']` 1424 s | `['B']` 1540 s | `['B']` 2130 s |
| 200 m | `['A','B']` 1284 s | — | — |

門檻：HERO 需 h_ceil ≥ 125 m，Pioneer 需 ≥ 141 m（= 120 + 20 + 起伏）。
把 `dh_band_min_m` 放寬到 ≤18 m 可讓 HERO 在 120 m 下可用，
但 Pioneer 的帶寬是 0，**放寬到 1 m 都救不回來**。

⇒ **在民航法規上限下，Pioneer PNP 結構上飛不了固定翼測繪。**
這不是程式的錯，是平台參數表的 `min_fw_operation_agl_m` 就等於法規上限。

### 二、目標 GSD 決定「相機的高度上限 h_max」（獨立的另一條）

```
h_max = GSD_req · f / p                                    # CameraSensor.agl_window
gsd_attainable ⇔ h_max ≥ h_minfw + dh_band_min_m           # platform.py:739
```

本專案單光相機（ADR-0009）f = 16 mm、像素間距 3.9 µm ⇒ `h_max ≈ 4102.6 × GSD`。
Pioneer 需 `h_max ≥ 140 m` ⇒ **GSD ≥ 0.0342 m/px**。

而 `mission/swarm_coordinator.py` 的 HV 分支原本**沒有傳 `payload_reqs`**，
轉接層就自己合成 `gsd = 1.05 · h_target · p / f`（`aeroplan_adapter.py:1011`），
使 `h_max = 1.05 × 作業高度`。作業高度 120 m ⇒ h_max 只有 126 m < 140
⇒ Pioneer 分不到工作，畫面只顯示「1/2 架」。

面板本來就有「目標 GSD」欄位（預設 0.05 m/px），只是從來沒被送下去。

實測（h_ceil = 200 m、作業高度 120 m、1A+1B）：

| 目標 GSD | h_max | 上工 | makespan |
|---|---|---|---|
| 未送（合成 1.05×h = 126 m） | 126 m | `['B']` | 2085 s |
| 0.05 m/px（面板預設） | 205 m | `['A','B']` | **1284 s** |
| 0.03 m/px | 123 m | `['B']` | 2085 s |

0.03 那一列是反向臂：修法不是「無論如何都讓 A 上工」，而是把真正的約束接上來。

## Decision / 決策

1. **豁免改成明示。** 面板加一個「已取得 AGL 豁免」核取方塊，預設**不勾**；
   不勾時 `get_params()` 回 `hv_waiver_max_agl_m = None`，
   `SwarmCoordinator` 原樣傳下去 ⇒ h_ceil = 法規的 120 m。
   勾了才啟用旁邊的數值欄（範圍 121–500，預設 200）。
   `_hv_get(..., default=200.0)` 一併改成 `default=None`。
2. **把面板的 `target_gsd` 包成 `PayloadRequirement` 傳給規劃器**，
   不再讓轉接層合成 `1.05×h`。同一個需求帶著還原後的 side overlap 與
   `h_target_agl_m`。
3. **規劃前先算高度帶並說出來。** 上游的 NO_GO 只寫
   「所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）」，四個原因擠在一句話。
   HV 分支在建 coordinator 之前逐構型印出 `h_ceil − max(h_minfw, h_clr)` 的算式，
   不足者走 `altitude_band_too_narrow` 前綴（已加進 `_HV_LOUD_NOTES`），
   「剛好等於下限」的也一併點名（地形起伏會吃掉）。
4. `partial_fleet_assigned` 的提示只在構型 A 真的閒置時才給，
   而且指向「填目標 GSD」而不是「調高作業高度」（後者在 120 m 下已違規）。

## Consequences / 後果

- **出廠組態現在會 NO_GO**，而且會明說原因。這是正確的：沒有豁免時這支機隊
  真的飛不了。使用者要嘛申請 ≥141 m 豁免（兩型都能用），要嘛只飛 HERO 並把
  `dh_band_min_m` 放寬到 ≤18（可用 `hv_params['hv_cfg']` 覆寫），
  要嘛換一台最低作業高度低於 100 m 的機體。
- 所有既有的 HV 呼叫端（測試、SITL 腳本、對照圖腳本）都必須**明給**豁免值。
  `tests/test_hv_dccpp_adapter.py` 的共用工廠已補上 200.0 並註明理由。
- 兩型同時上工時 makespan 由 2085 s 降到 1284 s（−38.4 %），
  代價是總航程 +9.7 %（多一台的進場／返場）。

## 驗證

`tests/test_hv_uniform_sensor.py`（49 條全綠）：

- `test_pioneer_is_unusable_at_120m_without_the_panel_gsd` —— 現象本身。
- `test_panel_gsd_makes_the_pioneer_usable_at_the_120m_legal_ceiling` —— 修法生效。
- `test_too_strict_gsd_still_excludes_the_pioneer` —— 反向臂。
- `test_panel_defaults_assume_no_waiver` —— 預設是 None，勾了才 ≥141。
- `test_no_waiver_means_the_legal_120m_ceiling` —— 擋住「退回 200」的回歸。
- `test_narrow_altitude_band_is_reported` —— 診斷訊息真的會叫。
- `test_wide_enough_ceiling_stays_quiet` —— 反向臂，證明它不是恆真。

⚠ `test_panel_defaults_*` 原本寫成 `QApplication.instance() or QApplication([])`
把回傳值丟掉，PyQt6 會在收尾時先回收 QApplication、再回收還活著的面板 ⇒ 整個
process 以 **exit 127 無聲死掉**（43 條綠燈後沒有 summary、沒有 traceback）。
已改成 session 範圍的 `qapp` fixture 留住參考。
