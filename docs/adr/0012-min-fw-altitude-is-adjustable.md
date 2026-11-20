# ADR-0012: 最低固定翼作業高度是可調的規劃值，不是機體極限

**狀態**: Accepted
**日期**: 2026-09-08
**決策者**: @james zheng
**相關**: [ADR-0011](0011-caa-120m-ceiling.md)

## Context / 背景

ADR-0011 的結論是「在民航法規 120 m AGL 上限下，Pioneer PNP 結構上飛不了固定翼
測繪」，理由是表 1 給它的 `min_fw_operation_agl_m` 就等於法規上限 ⇒ 可用高度帶為 0。

使用者質疑這個前提：「Pioneer 不應該被硬性規定最低固定翼高度是 120 m，
應該飛個 100 也可以吧」。查證之後**這個質疑是對的**。

### 一、120 這個數字沒有任何外部出處

`docs/hv_dccpp/平台參數表.md:34` 逐字：

```
| 高度 | min_fw_operation_agl_m | m AGL | 120 | 100 | 任務前需依地形、感測器與法規調整 |
```

- 使用方式欄自己就寫著**「任務前需依地形、感測器與法規調整」**。法規正是其中之一。
- 它**沒有**像旁邊兩列那樣標「暫定 placeholder，待實測覆寫」
  （`transition_start_agl_m` 60/50、`transition_complete_agl_m` 80–100/70–90 都有標），
  也沒有像其他列那樣標「公開最大航跡爬升角」「平台能力宣稱」這類出處。
- 設計文件 §17.1 記它的來源是「T1」（表 1 本身），全 repo 找不到原廠、
  datasheet 或法規引用。它是使用者給的**規劃值**。

### 二、規劃器自己就讓飛機飛在它以下

規格 D35 明文 `h_minfw` **只對 ENTRY/OPER/TRANS/RTB 生效**：

- CLIMB/DESCENT 段的 `z_lo` 不含 `h_minfw`（設計文件 Eq.59, :1242）。
- ACCEL/DECEL 段落在 `H_tc`，Pioneer 是 **80–100 m**，本來就低於 120。
- 設計文件 D.12 記著反例：舊版對所有段套 `h_minfw`，結果「平地、無風的最簡情境
  每架次都 INFEASIBLE」。

全 repo 沒有任何一句話描述低於它的機體後果（失速、操縱權限、適航）。
它是**規劃層的作業高度地板**，不是飛行包絡限制。

### 三、但它有真實下游

| 下游 | 位置 | 何時生效 | 影響 |
|---|---|---|---|
| `Q_RTL_ALT` | `hv_dccpp/ardupilot.py:922` | **每條路徑**（進匯出的 `.param`） | QRTL 進場高度 |
| `FENCE_ALT_MIN` | `hv_dccpp/mavlink_link.py:996` | 只在 `upload_plan(write_params/set_params)` | 飛機的圍籬地板 |

⚠ **2026-09-08 訂正**：原本寫成「兩個一起降」是錯的。`FENCE_ALT_MIN` 那一行雖然在
函式內無條件執行，但整個 `suggest_params` 只被 `mavlink_link.upload_plan` 呼叫；
AeroPlan 的 SITL 流程用 `export_all` 產 `.param` 再走 `--defaults`，只上傳任務，
**不走那條路** ⇒ `FENCE_ALT_MIN` 根本不會送出。HV4SITL8 的 `A1.param` 實測
`grep -c FENCE_ALT_MIN` = 0，而 `Q_RTL_ALT,100` 在。

### 四、上游只允許調高

`hv_dccpp/context.py:285`：`min_fw_operation_agl_m ... 只允許調高（現值 {old}）`，
低於現值直接丟 `MissionInputError`。所以 `platform_overrides` 這條路走不通。

## Decision / 決策

1. **走轉接層自己的 `platform_a=` / `platform_b=` 參數**
   （`aeroplan_adapter.py:1653`，於 `to_mission_inputs` :861 生效）傳入
   `dataclasses.replace` 過的 `PlatformSpec`。
   **權威表一個字都不改**，vendored 套件也不改。
2. 面板新增「最低固定翼作業高度」A／B 兩個欄位，**預設 A = 100、B = 100**。
   下界綁在各自的**轉換完成高度上緣**（A 100、B 90）：再低就是要求飛機在轉換
   完成前用固定翼巡航，那沒有依據。低於下界在面板擋、在協調器再擋一次
   （丟 `ValueError`）。
3. 面板新增「高度帶下限」欄位，**預設 15**（規格預設 20）。
4. 調低任一項都發 note：`min_fw_lowered(...)` / `dh_band_min_relaxed(...)`，
   兩者都進 `_HV_LOUD_NOTES`。前者明講 `Q_RTL_ALT` 會跟著降、`FENCE_ALT_MIN`
   只在走 `mavlink_link` 寫參數那條路時才送出；後者明講 GSD 可達性判準與
   相機預設目標高會一起放寬。

### 為什麼還要動高度帶下限

實測（2.45 km²、h_ceil = 120 m 法規值、作業高度 120、GSD 0.05、風 0）：

| min_fw A | min_fw B | 高度帶下限 | 上工 | makespan |
|---|---|---|---|---|
| 120（表值） | 100（表值） | 20（規格值） | **NO_GO** | — |
| **100** | 100 | 20 | **仍 NO_GO** | — |
| 105 / 110 / 120 | 100 | 20 | NO_GO | — |
| 99 | 99 | 20 | `['A','B']` | 1424 s |
| **100** | **100** | **15** | **`['A','B']`** | **1424 s** |
| 100 | 100 | 18 | `['A','B']` | 1424 s |

⇒ **只把 Pioneer 調到 100 是不夠的。** 120 − 100 = 20 恰好等於高度帶下限，
容許的地形起伏是 0，而本區的起伏不到 1 m 就足以讓它 NO_GO。
Pioneer 的合法下界就是 100（轉換完成高度上緣），所以合法範圍內**沒有**
任何 `min_fw` 解得掉，唯一的槓桿是高度帶下限。

規格 §4.6 說那 20 m 是「坡度包絡在起伏地形上需要的活動空間」，來源標
**D（本文預設）**而非任務輸入，而且全部文件都沒有寫出它為什麼是 20。
上游自己在 `h_ceil = 120` 的測試就用 15（`tests/hv_dccpp/test_preflight.py:354`）。
15 留 5 m 給起伏，是有先例且比 19 更能容忍實際地形的選擇。

## Consequences / 後果

- 出廠組態（不勾豁免、A/B 都 100、高度帶 15）在法規 120 m 上限下**可以規劃**，
  兩型都有工作，makespan 從表 1 值的 2085 s 降到 1424 s（−31.7 %）。
- ADR-0011 的結論**要訂正**：Pioneer 在 120 m 上限下飛不了測繪這件事，
  成立的前提是「沿用表 1 的 120」。表 1 自己就說那個值該依法規調整。
- 調低高度帶下限同時放寬了相機 GSD 可達性判準
  （`h_max ≥ 最低作業高度 + 15`，門檻由 0.0342 降到 0.0280 m/px）
  與相機預設目標高（`h_max − 15`）。兩者都是放寬，不會讓任務變得更冒險。
- `Q_RTL_ALT` 會從 120 降到 100（HV4SITL8 的 `A1.param` 讀回確認）。
  `FENCE_ALT_MIN` 只在走 `mavlink_link.upload_plan` 寫參數時才綁著它，
  目前的 SITL 與匯出流程都不走那條路。

## 驗證

`tests/test_min_fw_altitude.py`（14 條）：

- `test_the_authoritative_table_still_says_120_and_100` —— 值層級鎖，
  擋掉「乾脆去改表」這條路。
- `test_upstream_only_allows_raising_it` —— 記住為什麼不用 `platform_overrides`；
  哪天上游放寬就會紅。
- `test_panel_value_reaches_the_planner` —— **旋鈕真的送到規劃器**。
- `test_table_values_are_unflyable_at_the_legal_ceiling` —— 反向臂。
- `test_lowering_to_100_alone_is_still_not_enough` —— 擋掉「調成 100 就好」的直覺。
- `test_no_legal_min_fw_fixes_it_without_relaxing_the_band` —— 合法範圍內全掃過。
- `test_100_plus_relaxed_band_flies_both_types` —— 面板預設組合真的飛得起來。
- `test_below_transition_complete_is_refused` —— 下界守衛。
- `test_lowering_is_announced` / `test_table_values_stay_quiet` —— 正反向臂。

⚠⚠⚠ 第一版探針改的是 `hv_dccpp.platform.PLATFORM_PIONEER` 這個模組屬性，
而轉接層在**匯入時**就把它綁進 `_SPEC_OF`（:500）與 `platform_a` 的預設值（:1653）
⇒ 七個不同的 `min_fw` 跑出**一模一樣**的結果。那不是「沒有影響」，
是量不到東西。所以每一條會規劃的測試都先斷言旋鈕真的到得了規劃器。

⚠ 順帶修掉一個既有缺陷：規劃前發出的 note 一則一則送，而 `hv_on_report`
在許多消費端是 `dict.update` ⇒ 後送的會**整則蓋掉**先送的
（實測 `dh_band_min_relaxed` 吃掉了 `min_fw_lowered`）。改成收進 `_prenotes`
一次送出，並在尾端併進 `_vnotes`。
