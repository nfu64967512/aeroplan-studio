# VTOL 多機協同區域偵察 —— 實作交接說明書

> **給接手實作的人（Cowork）。**
> 來源：`曾維士學研獎助生研究計畫.odt`（學研獎助生每學期研究成果）。
> 落地日期：2026-09-09。落地方式：七個面向各派一個代理找程式碼證據，
> 再各由一個對抗性代理逐條複查，最後由主線親自重跑測試確認。
>
> **紀律**：本文只寫在 repo 裡查得到出處的東西。原報告中查不到出處的數字
> 已從正文移除，集中列在 **附錄 B**，不是被忘記，是刻意不採用。

---

## 0. 三十秒摘要

| | |
|---|---|
| 題目 | 垂直起降無人機（VTOL）多機協同區域偵察的航線規劃與飛行驗證 |
| 上位計畫 | 應用視覺里程計（VIO）在無 GNSS 信號之環境執行多旋翼無人機之定位與任務飛行 |
| 位置 | `C:\Users\joy46\Documents\Projects\aeroplan-studio`，分支 `quad_swarm`，HEAD `5e0295f` |
| 今日實測 | 20 個相關測試檔：**96 failed / 415 passed / 2 個檔收集 0 項** |

**最重要的一句話**：報告描述的六塊工作，**只有一半在現行工作樹上**。
另一半的產品程式碼不在樹上、也不在任何 branch／stash／worktree 上，
但**它們的測試還在，而且是紅的**。所以這份交接的核心工作不是「從零實作」，
而是**把那些紅色測試變綠**——規格已經寫好了，就在測試檔裡。

先讀 **§2**，再讀 **§7 的工作包**。其餘章節是背景。

---

## 1. 題目定位與兩套規劃器

### 1.1 一句話定位

在自己開發的 PyQt6 地面站 AeroPlan Studio 裡，把「多架 VTOL 協同掃描一塊區域」
從圈區一路做到模擬器實飛，並且**用飛出來的數字驗收，不用程式碼長相驗收**。

### 1.2 ⚠ 這個 repo 裡有**兩套**覆蓋規劃器，不要搞混

接手的人最容易踩的第一個坑。兩套平行存在、共用同一個 GUI 入口：

| | 既有 DCCPP（本報告的主體） | HARVEST（`hv_dccpp/`） |
|---|---|---|
| 位置 | `core/dccpp/` | `hv_dccpp/`（vendor 進來的獨立函式庫） |
| 機隊 | 同構，本報告用 3 架 | 異構雙構型（Pioneer PNP / HERO PNP） |
| 匯出 | `mission/vtol_mission_exporter.py` | `hv_dccpp/ardupilot.py` |
| 避撞 | `core/dccpp/collision_avoidance.py` | `hv_dccpp/deconflict.py` |
| 航次產物 | `exports/vtol_recon_run1`、`run2` | `runs/artifacts/HV4SITL2`～`HV4SITL13` |
| 資料型別 | 純 dict | 真 dataclass（`hv_dccpp/mission.py`） |
| 決策紀錄 | 無專屬 ADR | `docs/adr/0007`～`0014` |

**本報告講的是左欄。** `runs/artifacts/HV4SITL*` 那十幾趟航次是右欄的證據，
數字（4 機、120.1 m、32.43 km）不可以拿來當本報告成果的佐證。

兩者的唯一接縫是 `mission/swarm_coordinator.py::plan_coverage_dccpp`，
由「VTOL 偵察」分頁「5. 規劃」的下拉選單切換。

---

## 2. ⚠⚠⚠ 動手前必讀：現行工作樹狀態

### 2.1 哪些在、哪些不在

| # | 報告裡的工作 | 現況 | 硬證據 |
|---|---|---|---|
| 1 | 多機偵察操作頁面 | ✅ **在** | `ui/widgets/vtol_recon_panel.py`（7 個編號群組、19 個信號） |
| 2 | 相機覆蓋換算模組 | ✅ **在** | `core/recon/sensor_footprint.py` |
| 3 | 覆蓋率驗算模組 | ✅ **在** | `core/recon/coverage_verifier.py::verify_coverage` |
| 4 | VTOL 垂直起降剖面（缺陷②） | ✅ **在** | `core/dccpp/vtol_profile.py` + `DCCPPPathBuilder.prepend_vtol_takeoff` |
| 5 | 掃描方向可自訂 | ✅ **在** | `area_processor.width_along_direction:228`、`process_latlon(scan_angle_deg=)` |
| 6 | 上傳側速度修正（缺陷③後半） | ✅ **在** | `ui/main_window.py::resolve_upload_speed_radius` |
| 7 | **重疊率修正（缺陷①）** | ❌ **不在** | `area_processor.py:351` 仍 `step = w_i + w_ovl`；`:359` 仍 `+` |
| 8 | **匯出側速度型別（缺陷③前半）** | ❌ **不在** | `vtol_mission_exporter.py:135` 仍 `param1=1.0`（＝地速） |
| 9 | **ARMING_REQUIRE 注入（缺陷④）** | ❌ **不在** | `mission/sitl_launcher.py` 無 `VTOL_ENFORCED_PARAMS` |
| 10 | **聯合分派 joint / balance_load** | ❌ **不在** | `IDP_Solver.__init__()` 不收 `balance_load` |
| 11 | **避撞迭代收斂＋高度上限** | ❌ **不在** | `CollisionAvoidance.__init__()` 不收 `max_rounds` / `max_alt_offset_m` |
| 12 | 逐機轉彎半徑（報告未提） | ❌ **不在** | `ImportError: TurnRadiusUnusable`、`BuiltPath` 無 `turn_radius_m` |

### 2.2 這不是「還沒做」，是「做過但不在樹上」

四條互相獨立的證據：

1. **測試還在，而且比程式碼舊。** `tests/test_dccpp_strip_pitch.py`（2026-08-24）、
   `test_collision_alt_cap.py`（08-25）、`test_idp_load_balance.py`（09-04）、
   `test_per_aircraft_turn_radius.py`（09-05）都存在且逐條描述了預期行為。
2. **產品檔的 mtime 全部撞在同一刻。** `area_processor.py`、`collision_avoidance.py`、
   `vtol_mission_exporter.py` 三個檔的 mtime **都是 2026-09-06 21:24**，
   而測試檔全部早於它。像是一次批次還原，不是逐檔開發。
3. **從沒進過版控。** `git log --all -S"_SPEED_TYPE_AIRSPEED"` 與
   `-S"VTOL_ENFORCED_PARAMS"` 在**所有** ref 上都是 **0 個 commit**。
   HEAD 版的 `area_processor.py:296` 一樣是 `step = w_i + w_ovl`。
4. **新增檔活下來、修改檔沒活下來。** 全部 VTOL 偵察相關檔案都是 **untracked**。
   git 不碰 untracked 檔，所以新建的模組與測試都在；
   而對**已納管檔案**的修改被 checkout／restore 蓋掉了。

⭐ **唯一的復原線索**：`core/dccpp/__pycache__/idp_solver.cpython-310.pyc`
（22,281 bytes，2026-09-02 17:16）**是 joint 版的位元碼**，
內含 `balance_load` ×1、`joint` ×15、`contiguous` ×4、`minmax` ×4、`makespan` ×10。
對應的 `.py` 在任何地方都沒有。見 **WP-C**。
（`core/dccpp/idp_solver.py.bak` 是 4 月的舊 numpy 版，**沒有** joint，不是復原來源。）

### 2.3 ⚠ 有另一個 session 正在改同一棵樹

過去 6 小時內被改過的檔包含 `mission/sitl_launcher.py`（今天 15:08）、
`mission/swarm_coordinator.py`、`ui/main_window.py`、`ui/widgets/vtol_recon_panel.py`、
`hv_dccpp/` 六個檔、`tests/hv_dccpp/` 若干。

落地過程中就直接觀測到一次：`hv_dccpp/integration/aeroplan_adapter.py` 在
18:05:14 被改，害 `tests/test_hv_uniform_sensor.py::test_validate_is_actually_run`
（一條源碼字串鎖）從綠變紅。

**動手前先 `git status` 與 `ls -la` 比對 mtime，不要只看 `git status`。**
兩個 session 同時改同一個檔會互相覆蓋，而且不會有任何錯誤訊息。

---

## 3. 一站式資料流（八站）

「一站式管理」落地成一個分頁：`ui/widgets/vtol_recon_panel.py` 的 `VTOLReconPanel`，
七個編號群組（1 偵察區域 → 2 感測器足跡 → 3 VTOL 參數 → 4 群飛編組 → 5 規劃 →
6 覆蓋率驗算 → 7 輸出 / 模擬），另加兩個未編號群組：HARVEST（`_build_hv_group`）
與 Fence 管理（`_build_fence_group`）。
面板本身**不做任何副作用**，只發 19 個 `pyqtSignal`，由 `ui/main_window.py` 的
handler 執行。這個設計要保留：它是面板可以被離線測試的原因。

| 站 | 做什麼 | 檔案 / 入口 |
|---|---|---|
| ① 圈區 | 地圖點擊 → 角點 → 多邊形 | `ui/widgets/dual_map_widget.py` → `MainWindow.on_corner_added` / `survey_area_polygons` |
| ② 相機換算 | HFOV/影像寬/航高 ↔ 掃描寬度、GSD | `core/recon/sensor_footprint.py::compute_footprint` / `altitude_for_gsd_m` |
| ③ 分配 | 條帶分給各機 | `core/dccpp/task_allocator.py` → `core/dccpp/idp_solver.py` |
| ④ 避撞 | 時空衝突偵測與解衝突 | `core/dccpp/collision_avoidance.py::run` |
| ⑤ 覆蓋率驗算 | 規劃**之後**獨立再驗一次 | `core/recon/coverage_verifier.py::verify_coverage` |
| ⑥ 任務匯出 | QGC WPL 110 + 任務簡報 | `mission/vtol_mission_exporter.py::export_all` |
| ⑦ 上傳飛控 | MAVLink 上傳 | `MainWindow.on_sitl_upload_mission` |
| ⑧ SITL | 起 N 台 ArduPilot | `mission/sitl_launcher.py::start_multi` |

規劃的唯一接縫是 `mission/swarm_coordinator.py::plan_coverage_dccpp`，
回傳一個 6 鍵字典，存進 `MainWindow._dccpp_result`。

**要看懂資料結構，先讀這三個檔**：
`core/dccpp/dccpp_path_builder.py`（`BuiltWaypoint` / `BuiltPath` / `SegmentLabel`）、
`ui/widgets/vtol_recon_panel.py::get_params`（約 35 個鍵）、
`hv_dccpp/mission.py`（HARVEST 側的 dataclass）。

**CLI 現況**：`cli/` 只有 `sitl` 與 `demo` 兩個子指令，**沒有**跑完整條鏈的 CLI。
無頭跑法是 `scratchpad/acceptance_vtol_recon.py`（走產品路徑：
`VTOLReconPanel.get_params()` → `plan_coverage_dccpp()` → 取 OPERATION 段 →
`verify_coverage()`）。⚠ 它未被 git 追蹤。

---

## 4. 演算法規格

### 4.1 四個步驟

1. **決定掃描方向** —— 預設由 `find_min_width()` 取幾何最小寬度方向；
   可由面板釘死成指定方位角（`width_along_direction`，量的是**投影跨距**不是最小值）。
2. **切掃描帶** —— `generate_paper_scan_lines`（`area_processor.py:321`），
   實作論文 Wang et al. 2025（*Defence Technology* 47:197–215）的 Eq.(7)。
3. **分配給各機** —— 目標是讓最慢那架越早結束越好（min-max makespan）。
4. **Dubins 串接** —— 起飛／掃描／轉場三段接起來（`core/trajectory/dubins_trajectory.py`）。

### 4.2 論文沒有、本研究補上的兩件事

**（a）相機模組** `core/recon/sensor_footprint.py`
讓「影像要看多細（GSD）」與「飛多高、一趟掃多寬」可以互相換算。
公開 API：`compute_footprint` / `swath_width_m` / `gsd_m_per_px` /
`altitude_for_gsd_m` / `effective_sweep_width_m`。
掛載角上限硬編在 `_MAX_LOOK_ANGLE_DEG = 85.0`。

⚠ HARVEST 側是**另一套**相機模型（`hv_dccpp/platform.py`，用焦距與像素間距，
不是 FOV）。兩套並存，改一邊不會影響另一邊。

**（b）獨立覆蓋率驗算** `core/recon/coverage_verifier.py::verify_coverage`
規劃完之後把作業段航點網格化重算一次覆蓋率與漏掃格，
**不讓規劃程式自己說自己對**。

⚠⚠ 已知結構性缺口（未修）：驗算只吃 `(lat, lon)`，**從不讀航點的 `alt`**，
再配一個由面板航高算出的**單一** `sweep_width_m`。所以當各機實際高度不同時，
它回報的覆蓋率是對「假設全隊都飛設定高度」成立，不是對實際任務成立。
這正是缺陷⑤（高度被抬高）能躲過驗算的原因。**修高度問題時要順手補這個。**

---

## 5. 四個靜默缺陷

四個都屬於同一類：**不會報錯、log 看起來正常、但結果是錯的**。

### 缺陷① 掃描帶重疊率失效

**位置** `core/dccpp/area_processor.py:351-359`

現行程式碼：
```python
step = w_i + w_ovl                                   # :351
n_layers = max(1, int(math.ceil((W - w_i) / step)) + 1)
dw_top = (li - 1) * (w_i - w_ovl)                    # :358
dw_bot = li * w_i + (li - 1) * w_ovl                 # :359
```

中線 =（dw_top + dw_bot）/ 2 時 `w_ovl` 項**剛好對消**：

```
dw_mid(l) = ½[(l−1)(w_i−w_ovl) + l·w_i + (l−1)·w_ovl] = ½·w_i·(2l−1)
```

⇒ 中線間距恆等於 `w_i`，與重疊率無關。

**⚠⚠ 訂正報告的措辭**：報告寫「怎麼調結果都一樣」，**只對了一半**。
中線**間距**確實不變，但條帶**數量**會隨重疊率**遞減**（`:351` 的 `step` 是 `+`），
所以調高重疊率會讓覆蓋率**變差**，方向是反的。實測（2.0×1.5 km、w_i=110.88）：

| w_ovl | 條帶數 | 覆蓋率 |
|---|---|---|
| 0 | 13 | 97.00% |
| 20 | 12 | 90.00% |
| 40 | 11 | 82.00% |

只修中線公式而不修 `:351` 的 `step`，`n_layers` 仍然是錯的。
這個錯誤措辭同時存在於報告與 `tests/test_dccpp_strip_pitch.py:14` 的 docstring，
修的時候一起更正。

**修法** `dw_bot` 的 `+` 改 `−`（條帶寬度恆為 `w_i`、中線間距 = `w_i − w_ovl`），
`step` 由 `w_i + w_ovl` 改成 pitch `w_i − w_ovl`。
⚠ 論文印的是 `+`；無法斷定是原文誤植或 PDF 抽取損壞，**以物理自洽為準**。

**守衛** `tests/test_dccpp_strip_pitch.py`（值層級：直接量實際產生的中線間距，
不鎖公式長相）。→ **WP-A**

### 缺陷② 規劃器不認得 VTOL ✅ 已在樹上

**症狀** 選了 VTOL 仍規劃出約 500 m 的定翼跑道離場。

**根因** 真正的判別式是 `if u.turn_radius > 0`，不是 `vehicle_type`；
而 `vehicle_type='vtol'` 在 `swarm_coordinator` 被降級成 `VehicleType.MULTIROTOR`
（那個 enum 根本沒有 VTOL 成員）。QuadPlane 的 `turn_radius` 合法地大於 0
⇒ 被當固定翼。`plan_coverage_dccpp()` 當時連 `transition_alt` 這個參數都沒有，
起降參數是 `runway_bearing_deg` / `runway_length_m` / `landing_rollout_m`，
而 VTOL 偵察頁沒有跑道欄位 ⇒ `runway_length` 吃預設值。

**修法（已在樹上）** `core/dccpp/vtol_profile.py` 定義剖面幾何常數，
`DCCPPPathBuilder.prepend_vtol_takeoff` / `append_vtol_landing` 產生真垂直段：

```
Home（地面） ──純垂直──> 轉換高度 ──前向過渡──> 固定翼爬升 ──> 巡航高度
```

**⚠ 垂直段會弄壞三個下游**（都已修，接手時不要退回去）：
1. SITL 生成朝向只比對 `waypoints[1]`，同經緯度算不出方位 ⇒ 靜靜退回 0°。
   改成往後掃第一個有水平位移的點。
2. NFZ 避障後用**水平**弧長比例重採樣高度 ⇒ 垂直段佔比 0，整段爬升被擠掉。
   補 `_split_takeoff_head`（尾端 `_split_landing_tail` 的鏡像）。
3. 避撞的高度偏移窗是純索引式，沒有 TAKEOFF/LANDING 守衛 ⇒ 會把地面點抬高
   ⇒「在半空中降落」。

**守衛** `tests/test_vtol_profile.py`（43 條，今日全過）。
其中 `:222 test_vtol_takeoff_is_not_the_runway_takeoff` 是刪除實驗對照：
固定翼版第一段 > 50 m 位移 vs VTOL 版 == 0。

⚠ **仍未修**：`BuiltPath.total_length_m` = entry + operation + transfer，
**沒有 takeoff / landing**（`AssembledPath` 五項都有）⇒ `total_makespan` 系統性低估。
修它會同時踩紅 `test_dccpp_pipeline.py` 的兩條特徵化測試，兩條都要一起改。

### 缺陷③ 匯出與上傳是兩條路，各錯一半

從 SITL 讀回上傳的任務，和匯出的 `.waypoints` 比對：

| 項目 | 匯出檔 | 上傳飛控（讀回） |
|---|---|---|
| `DO_CHANGE_SPEED` param1 | **1.0 = 地速** | 0.0 = 空速 |
| `DO_CHANGE_SPEED` param2 | 18.0 m/s | **3.0 m/s** |
| `NAV_WAYPOINT` param2 | 60 m | 50 m |

MAVLink `common.xml` 的 `enum SPEED_TYPE`：**0 = Airspeed、1 = Groundspeed**。
⇒ 匯出檔設的是地速下限，根本沒設巡航空速；上傳側型別對但值是 3.0，
遠低於 QuadPlane 的 `AIRSPEED_MIN` ⇒ 會被夾掉。**兩條路都沒有真的設到巡航空速。**

**上傳側根因（已修）** 上傳路徑讀的是「基本演算法」分頁的
`parameter_panel.get_parameters()`，不是 VTOL 偵察頁；那個 panel 的 `speed`
從頭到尾沒被動過，還是初始預設 3.0。
修法：規劃時把 `cruise_speed_mps` / `turn_radius_m` 記進 `_dccpp_result`，
上傳端用 `resolve_upload_speed_radius()` 優先讀它。

**匯出側（未修）** `mission/vtol_mission_exporter.py:135` 仍是 `param1=1.0`，
而 `:131` 的註解寫「param1=1.0 (空速)」**是錯的**——同一個檔 `:39` 自己就寫著
`param1=速度類型(0=AS,1=GS)`。→ **WP-F**

### 缺陷④ QuadPlane 在 SITL 飛不起來

**症狀** 四軸馬達一直停在最低油門。

**根因** `sitl/default_params/plane.parm:84` 的 `ARMING_REQUIRE 0`。
判別法：PWM **0** = 該通道沒指派馬達功能；PWM **1000** = 有指派但被命令關掉。
兩者意義完全不同。
⚠ 另一個錯誤直覺：`--model quadplane` **只決定物理模型**，QuadPlane 功能要靠參數開，
`--model quadplane` ≠ `Q_ENABLE=1`。

**修法：注入，不是檢查。**
對 `vehicle == 'VTOL'` 產生 `vtol_enforced.parm` 並掛進 `--defaults` 鏈：

```
Q_ENABLE 1 / Q_FRAME_CLASS 1 / Q_FRAME_TYPE 1
SERVO5~8_FUNCTION 33~36
ARMING_REQUIRE 1        ← 覆蓋 plane.parm 的 0
```

關鍵是**注入**不是檢查：檢查只會事後告訴你壞了；注入直接讓它是對的，
而且不依賴任何人維護 `plane.parm`。

**⚠ 排序有實質意義。** `--defaults` 後面覆蓋前面。
目前唯一真的在注入的地方是 `scripts/hv_sitl_flight.py:176-189`，六份的順序是：
`plane.parm → vtol_default.parm → identity.parm → vtol_enforced.parm → <UAV>.param → firmware_override.parm`
——強制檔排**第 4**（倒數第三），`<UAV>.param` 排在它**後面**。
今天安全（那些 `.param` 只有 `Q_ENABLE,1`），但只要哪天含 `ARMING_REQUIRE` 就會被蓋掉。
`scripts/hv_sitl_flight.py:19` 的 docstring 只列五份，已過時，不要照抄。

**⚠⚠ 產品啟動器沒有這個。** `mission/sitl_launcher.py` 完全沒有
`VTOL_ENFORCED_PARAMS`，`:571` 寫的 `SCHED_LOOP_RATE=50`（測試期待 200）。
`scripts/hv_sitl_flight.py:14` 自己就逐字寫著「那一版 launcher 不會掛上去」。
⇒ 走 GUI 產品路徑起 SITL，**現在仍然會踩到這個 bug**。→ **WP-E**

**順帶三個 PreArm 修正**（其中一個已在樹上）：
1. *VTOL land too short* —— 飛控用**它自己的** `AIRSPEED_CRUISE` 算煞停距離，
   **跟面板速度無關**。修法是把 `land_approach_m` 取
   `max(back_transition, 飛控煞停距離 × 1.25)`。
2. *Gyro rate* —— SITL 模擬陀螺儀 400 Hz，`300 × 1.8 = 540 > 400` 恆擋
   ⇒ loop rate 300 → **200**。（❌ 現行 launcher 是 50）
3. *真解鎖優先*（✅ **已在樹上**）—— `mission/sitl_link.py:865 _arm_with_prearm`
   先發正常 ARM（param2=0）吃真 PreArm，被擋才 force(21196) 並大聲警告。
   ⚠ 但產品呼叫點目前全部傳 `force_fallback=True`
   ⇒「0 次強制解鎖」是那一趟的**觀測結果**，不是結構保證。

---

## 6. 驗證方法論（這是本研究最可複製的部分）

四條紀律，寫在 `docs/design_2026-09-05_heterogeneous_vtol_planning.md` §四：

1. **值層級鎖，不是源碼層級鎖。**
   量「實際產生的中線間距是多少」，不是 grep「那一行有沒有寫 `w_ovl`」。
   源碼鎖驗的是那一行長什麼樣，改上游／下游／旁路全部免疫，補鎖是軍備競賽。
2. **可觸發性前置斷言。** 每條測試先斷言「這個設定真的能讓 X 發生」，
   再斷言 X 的值。否則「量到 0 次」與「量測器根本不可能觸發」長得一模一樣。
3. **突變驗證。** 故意把程式改壞，確認測試真的會轉紅。
   臂數不夠不算數，要涵蓋**靜默失效**臂。
4. **逐檔 `--collect-only`。** 防「收集 0 項卻不報錯」的假測試。

### 突變測試怎麼跑

`scratchpad/bite_*.py` 共 16 個檔，其中 `bite_lock.py` 是檔案鎖本身，
實際突變臂腳本 **15 支**（合計約 224 臂）。

結構（以 `bite_fov_cone.py` 為範本）：
```
ARMS = [(名稱, 檔案路徑, old 片段, new 片段), ...]
拿 BiteLock → 逐臂就地改寫源碼 → subprocess 跑 pytest → finally 還原
→ anchor 命中次數 != 1 就 SKIP → rc 非 0 記 BITES，否則 SURVIVED
```

**⚠⚠⚠ 三個必須知道的陷阱：**

* **突變腳本會就地改寫生產源碼。** 同時跑兩份會把突變留在樹上，
  其中有些不會報錯只會靜靜飛錯。`scratchpad/bite_lock.py` 是為此加的互斥鎖
  （寫入 pid + 時間，拿不到就 `sys.exit` 不排隊）。**一定要透過它跑。**
* **對照組不綠就不要跑突變。** 對照組本來就紅的話，每一臂都會「看起來被咬到」，
  全部是假陽性。今天 `bite_alt_cap.py` 的對照組是 20 紅
  ⇒ 它的 17 個可套臂會產生 17 個假陽性，**現在不要跑它**。
* **anchor 會失效。** `bite_alt_cap.py` 的 23 臂裡有 6 臂的 anchor 在現行
  `collision_avoidance.py` 命中 0 次（因為那些程式碼不在樹上）⇒ 會 SKIP。

今天唯一安全可重跑的是 `scratchpad/bite_fov_cone.py`
（17/17 臂 anchor 齊全，對照組 117 passed / 0 failed）。

### 已知的假測試

`tests/test_heterogeneous_nfz_planner.py` —— 被 git 追蹤、檔名符合 `test_*.py`、
pytest 收集 **0 項**且**不報任何錯**（rc=5）。整檔 0 個 `def test_`，
實際是 `if __name__ == '__main__': sys.exit(run_all())` 的腳本。

---

## 7. 工作包

六個工作包互相獨立，可以並行。**每一包的驗收標準都是「那些測試變綠」。**

共同前置：
```bash
cd /c/Users/joy46/Documents/Projects/aeroplan-studio
git status                     # 確認沒有別的 session 正在改你要動的檔
python -m pytest tests/test_vtol_profile.py -q -o addopts=""   # 對照組應 43 passed
```

⚠ `pytest.ini` 的 `addopts` 含 `-v`，會蓋掉命令列的 `-q`。
所以下面每條指令都帶 `-o addopts=""`。

---

### WP-A　條帶重疊率（缺陷①）

| | |
|---|---|
| 現況 | `tests/test_dccpp_strip_pitch.py` **11 failed / 7 passed** |
| 要改 | `core/dccpp/area_processor.py:351`、`:359` |
| 失敗訊息 | `AssertionError: 條帶寬度應該 100.0，實際出現 140.0（w_ovl=20.0）`、`重疊率 20% 沒有讓條帶變密（20 → 17）` |

做法：`:359` 的 `dw_bot = li*w_i + (li-1)*w_ovl` 改成 `−`；
`:351` 的 `step = w_i + w_ovl` 改成 `w_i - w_ovl`。兩處都要改。

⚠ `tests/test_dccpp_pipeline.py` 是特徵化測試，黃金值會跟著變
（它檔頭本來就寫明「刻意更改演算法行為必須同步更新」）。改完要重跑並更新黃金值。
⚠ VTOL 偵察面板**不受這個 bug 影響**：它把重疊折進 `fov_width_m` 並送
`overlap_rate=0`，歪打正著避開了。受影響的是 **DCCPP 分頁**（預設重疊 0.1）。
⚠ 順手把 `test_dccpp_strip_pitch.py:14` 與 `:73` 的 docstring 措辭改對（見 §5 缺陷①）。

**驗收**
```bash
python -m pytest tests/test_dccpp_strip_pitch.py tests/test_dccpp_pipeline.py -q -o addopts=""
```

---

### WP-B　逐機轉彎半徑（報告未提，但測試已寫好）

| | |
|---|---|
| 現況 | `tests/test_per_aircraft_turn_radius.py` **24 failed** |
| 要改 | `core/dccpp/idp_solver.py`、`core/dccpp/dccpp_path_builder.py` |

失敗訊息聚類：
```
ImportError: cannot import name 'TurnRadiusUnusable' from 'core.dccpp.idp_solver'   ×10
AttributeError: 'UAVPlan' object has no attribute 'turn_radius_m'                    ×6
AttributeError: 'BuiltPath' object has no attribute 'turn_radius_m'                  ×6
TypeError: DCCPPPathBuilder.build() got an unexpected keyword argument 'turn_radius'  ×2
AttributeError: 'DCCPPPathBuilder' object has no attribute '_dubins_for'              ×4
AttributeError: 'IDP_Solver' object has no attribute 'cost_cache_for'                 ×4
```

背景：全隊共用單一轉彎半徑，用 200 m 規劃小機過度保守、用 120 m 規劃大機直接飛不出來
（`docs/adr/0007` 有完整論述）。要新增的是逐機半徑 + 一個 `TurnRadiusUnusable` 例外
+ 逐半徑的成本快取。

**驗收**
```bash
python -m pytest tests/test_per_aircraft_turn_radius.py -q -o addopts=""
```

---

### WP-C　聯合分派（joint / balance_load）

| | |
|---|---|
| 現況 | `tests/test_idp_load_balance.py` **24 failed**；`tests/test_solver_wallclock_independence.py` **15 failed / 4 passed** |
| 要改 | `core/dccpp/idp_solver.py` |
| 失敗訊息 | `TypeError: IDP_Solver.__init__() got an unexpected keyword argument 'balance_load'` |

**⭐ 先做這件事：從位元碼復原。**
```bash
ls -la core/dccpp/__pycache__/idp_solver.cpython-310.pyc   # 22281 bytes, 2026-09-02 17:16
python -c "import dis,marshal,importlib.util as u; f=open('core/dccpp/__pycache__/idp_solver.cpython-310.pyc','rb'); f.read(16); dis.dis(marshal.load(f))"
```
它是 Python **3.10** 位元碼（本機是 3.14），不能直接 import，但可以 `dis` 讀出結構，
或用 `decompyle3` / `pycdc` 反編譯。裡面完整保留了設計說明：
以連續切塊的平衡解為起點 → 反覆針對瓶頸那架做搬移／交換 → 只接受嚴格下降。

設計上是三級，以 `assignment=` 切換，逐級以前一級為起點且只接受嚴格下降
⇒ 保證 `joint ≤ contiguous ≤ index`：

* `index` —— 最舊的等索引切塊
* `contiguous` —— 連續切塊 + 邊界爬山 + 區塊↔UAV 窮舉
* `joint`（預設）—— 論文 §3.3 聯合 MDTSP，放棄連續限制，局部搜尋逼近 minmax

**⚠⚠ 做完 WP-C 一定要接著做 WP-D。** joint 會產生**交錯**指派
（例如 UAV3 拿 [6,8,10]、UAV4 拿 [7,9,11]），兩架因此同時飛在相鄰條帶上，
間距會掉到 1.6 m。**單獨上 joint 而不修避撞，是把安全問題引進來。**

⚠ 牆鐘預算要用**確定性**預算（評估次數），不要用秒數——
`test_solver_wallclock_independence.py` 鎖的就是這件事。

**驗收**
```bash
python -m pytest tests/test_idp_load_balance.py tests/test_solver_wallclock_independence.py -q -o addopts=""
```

---

### WP-D　避撞迭代收斂 + 高度上限

| | |
|---|---|
| 現況 | `tests/test_collision_alt_cap.py` **16 failed**、`tests/test_collision_avoidance_rounds.py` **4 failed** |
| 要改 | `core/dccpp/collision_avoidance.py` |

失敗訊息：
```
TypeError: CollisionAvoidance.__init__() got an unexpected keyword argument 'max_alt_offset_m'   ×24
TypeError: CollisionAvoidance.__init__() got an unexpected keyword argument 'max_rounds'          ×2
AttributeError: 'CollisionAvoidance' object has no attribute '_effective_cap'                     ×8
```

現行 `run()`（`:456`）只做**一輪**：`detect` → `resolve` → 結束。

**要做四件事，缺一不可：**

1. **迭代到收斂**（加 `max_rounds`）。單輪解不完。
2. **高度累加上限**（加 `max_alt_offset_m` / `_effective_cap`）。
   沒有上限時每輪再抬一次，`120 + 10×30 = 360` m 就是這樣來的
   （機制寫在 `tests/test_collision_alt_cap.py:5-10`）。
3. **停滯保險絲要真的能燒斷。** 原本 `_resolve_altitude` 先無條件
   `actions.append(...)`，之後才在套用迴圈裡對 TAKEOFF/LANDING `continue` 跳過
   ——而 t=0 的衝突正好就在那些地面點上 ⇒ `new_actions` 永遠非空
   ⇒ `if not new_actions` 的停滯判斷**永遠不會觸發** ⇒ 一定跑滿。
   修法：在迴圈**一開頭**就跳過「衝突點本身在起降段」的衝突，不排偏移窗也不記動作。
   ⚠ 只在事後過濾「動作有沒有動到航點」不夠——偏移窗仍會蓋到作業航點。
   判準必須是**衝突點本身**動不動得了。
4. **撞到上限要大聲。** 靜靜停下來和靜靜飛到 360 m 一樣糟
   （`test_collision_alt_cap.py:107` 鎖的就是這條）。

**⚠ 另外兩個獨立的坑（都在 GUI 側，不在避撞器裡）：**

* **生成點間距不能等於門檻。** 生成點用 `111320.0` m/deg 排，
  而 `CollisionAvoidance._haversine` 用 `EARTH_RADIUS_M = 6378137`
  ⇒ `2πR/360 = 111319.4908` m/deg。50.0 m 名目間距被 haversine 量成
  **49.999771 m**，比門檻低 0.23 mm ⇒ `if dist < min_sep` 在 t=0 對每一組
  相鄰機**必定**成立，而且 cos(緯度) 在兩邊約掉 ⇒ 與緯度無關、必然發生。
  衝突本身是假的。修法：間距改 `max(50, min_sep × 1.2)`，
  且排點與量距改用**同一個**地球半徑常數。**兩件都要做**，只換常數仍會壓在刀鋒上。
* **`alt_offset_m < min_separation_m` 是無解組態。** 面板的 `get_params()`
  沒有 `alt_offset_m` 這個鍵 ⇒ 永遠退回預設 30，而 `min_separation_m` 送的是 50。
  30 < 50 ⇒ 純高度錯開在數學上解不開。

* **`temporal` 策略原本是完全 no-op**：它插入的 loiter 航點與參考航點同經緯度同高度，
  而時間由「累積距離 / 速度」推出 ⇒ 零長度線段貢獻 0 秒延遲；
  `AvoidanceAction.delay_s` 沒有任何下游會消費。已改為大聲報錯並退回 altitude。

* ⚠ **避撞目前只在 GUI 路徑跑**，headless／匯出路徑沒有保護。這是待解項。

**驗收**
```bash
python -m pytest tests/test_collision_alt_cap.py tests/test_collision_avoidance_rounds.py -q -o addopts=""
```
另外做一次端到端比對：重跑規劃並匯出，三架 `.waypoints` 的最大高度應該
全部等於設定航高（對照 `exports/vtol_recon_run1`（壞：120/360/360）
與 `exports/vtol_recon_run2`（好：三架全 120））。

---

### WP-E　SITL 機體參數注入（缺陷④）

| | |
|---|---|
| 現況 | `tests/test_sitl_airframe_params.py` **14 failed**、`tests/test_vtol_prearm_real.py` **3 failed**、`tests/test_vtol_sitl_enforced.py` **收集 0 項** |
| 要改 | `mission/sitl_launcher.py` |

失敗訊息：
```
TypeError: generate_vtol_params() got an unexpected keyword argument 'airframe'   ×24
ImportError: cannot import name '_SCHED_LOOP_RATE' from 'mission.sitl_launcher'   ×4
ImportError: cannot import name 'identity_parm_text' from 'mission.sitl_launcher' ×2
ImportError: cannot import name 'VTOL_ENFORCED_PARAMS' (test_vtol_sitl_enforced 整檔)
```

要新增的 API（由測試反推）：
`VTOL_ENFORCED_PARAMS`、`vtol_enforced_parm_text()`、`write_vtol_enforced_parm()`、
`build_defaults_list()`、`identity_parm_text()`、`_SCHED_LOOP_RATE`（VTOL 要 200，
現行 `:571` 是 50），以及 `generate_vtol_params(airframe=...)`。

實作要點見 §5 缺陷④。**強制檔必須排在 `--defaults` 鏈的最後**，
並保留兩條對照組測試：「`plane.parm` 本身沒有 `Q_ENABLE`」與
「`plane.parm` 仍寫著 `ARMING_REQUIRE 0`」（`sitl/default_params/plane.parm:84`）
——它們證明「排最後覆蓋」不是多餘的。

⚠ `mission/sitl_launcher.py` **今天 15:08 才被另一個 session 改過**，動手前先看 diff。
⚠ ArduPilot 4.7/4.8 對 `--defaults` 裡的未知參數名是**靜默略過**的，
所以參數名寫錯不會有任何錯誤訊息。務必用讀回式驗證
（範本：`scripts/hv_sitl_run.py:61-69` 的 `CHECK_PARAMS` / `EXPECT_EXTRA`，
產物格式見 `runs/artifacts/HV4SITL13/params_readback.csv`）。

**驗收**
```bash
python -m pytest tests/test_sitl_airframe_params.py tests/test_vtol_prearm_real.py tests/test_vtol_sitl_enforced.py -q -o addopts="" --continue-on-collection-errors
python scratchpad/acceptance_vtol_sitl.py     # 起 3 台，pymavlink 讀回 Q_ENABLE / SCHED_LOOP_RATE 再試 ARM
```

---

### WP-F　匯出側速度型別（缺陷③前半）

| | |
|---|---|
| 現況 | `tests/test_vtol_recon_fixes.py` **收集 0 項**（`ImportError: _SPEED_TYPE_AIRSPEED`） |
| 要改 | `mission/vtol_mission_exporter.py:131`、`:135` |

最小改動：新增常數 `_SPEED_TYPE_AIRSPEED = 0`（與 `_SPEED_TYPE_GROUNDSPEED = 1`），
`:135` 的 `param1=1.0` 改成 `param1=float(_SPEED_TYPE_AIRSPEED)`，
`:131` 那行寫錯的註解一起改對。

⚠ `tests/test_vtol_recon_fixes.py` 是 50 條的檔，還鎖了其他四件事
（生成點間距、避撞不抬起降段、上傳速度來源、3D 守衛），
所以它變綠需要 WP-D 一起完成。可以先讓它能被收集，再逐條轉綠。

⚠ 該檔用的 `_ca(max_rounds=...)` 依賴 WP-D 的新簽章。

**驗收**
```bash
python -m pytest tests/test_vtol_recon_fixes.py -q -o addopts=""
```
另外重新匯出一次，檢查 `.waypoints` 第 2 行的 `param1` 應為 `0.0`
（對照現況 `exports/vtol_recon_run1` / `run2` 都是舊行為）。

---

### 全部做完之後

```bash
python -m pytest tests/test_collision_alt_cap.py tests/test_collision_avoidance_rounds.py \
  tests/test_dccpp_coverage_retention.py tests/test_dccpp_pipeline.py tests/test_dccpp_reroute_altitude.py \
  tests/test_dccpp_return_leg_nfz.py tests/test_dccpp_scan_angle.py tests/test_dccpp_strip_pitch.py \
  tests/test_fence_scan_angle.py tests/test_idp_load_balance.py tests/test_recon_coverage.py \
  tests/test_vtol_fov_cone.py tests/test_vtol_mission_export_commands.py tests/test_vtol_prearm_real.py \
  tests/test_vtol_profile.py tests/test_vtol_recon_fixes.py tests/test_vtol_recon_panel.py \
  tests/test_vtol_sitl_enforced.py tests/test_per_aircraft_turn_radius.py tests/test_sitl_airframe_params.py \
  -q -o addopts="" --continue-on-collection-errors -p no:randomly
```
今日基準線：**96 failed / 415 passed / 2 errors，14.08 s**。目標 0 failed / 0 errors。

**⚠ 最後一步：把這些工作 commit 進版控。**
這次的資料遺失就是因為全部是 untracked 或未提交的修改。

---

## 8. 證據對照表（只列查得到出處的）

| 主張 | 數字 | 出處 | 可重跑？ |
|---|---|---|---|
| 高度被靜默放大（修正前） | UAV1 120 m / UAV2 360、240 m / UAV3 360 m | `exports/vtol_recon_run1/*.waypoints`（2026-08-25 00:31，逐筆可讀） | ❌ 需先做 WP-D |
| 高度修正後 | 三架 `.waypoints` 最大高度全部 120.00 | `exports/vtol_recon_run2/*.waypoints`（01:55） | ❌ 同上 |
| 掃描寬度與地面解析度 | 120 m 航高 ⇒ 刈幅 138.6 m、中心 GSD 3.46 cm/px | `tests/test_collision_alt_cap.py:10` | ✅ `core/recon/sensor_footprint.py` 可算 |
| 條帶間距缺陷的量化 | 2.0×1.5 km 覆蓋率卡在 97.0% | `tests/test_dccpp_strip_pitch.py:123`（該測試實跑印出 97.00%，9700/10000 格） | ✅ |
| 重疊率完全失效 | w_i=100、w_ovl=20 應得間距 80，實測 100；w_ovl=40 應得 60，實測 100 | `tests/test_dccpp_strip_pitch.py:6-13` | ✅ |
| 交錯指派的近距接觸 | 6 架、2.0×1.5 km、門檻 50 m：避撞前 1.6 m → 單輪 32.4 m（2 殘留）→ 迭代收斂 60.2 m（0 殘留） | `tests/test_collision_avoidance_rounds.py:6-17` docstring；`exports/vtol_progress_report_2026-08-25.pptx` slide 5 | ❌ 需 WP-C + WP-D |
| VTOL 剖面真的是垂直的 | 起飛第一段水平位移 == 0（固定翼版 > 50 m） | `tests/test_vtol_profile.py:222` | ✅ 43 條今日全過 |
| 掃描方向可自訂 | 手動填入自動回報的角度 ⇒ 逐點重現自動結果 | `tests/test_dccpp_scan_angle.py`（25 條，今日全過） | ✅ |
| 匯出產物 | 3 架、120 m、轉換高度 40 m、巡航 18 m/s、轉彎半徑 60 m；每架 15 原始航點；總長 1691 / 1618 / 1530 m | `exports/vtol_recon_run2/VTOL_briefing.txt` | ✅ 檔案在 |
| 任務上傳 | 13/13 ACCEPTED、0 次強制解鎖 | `exports/vtol_progress_report_2026-08-25.pptx` slide 5 | ❌ 見下方三條註記 |

**⚠ 關於「13 筆 ACCEPTED」的三條註記，寫論文或報告時必須帶上：**

1. **不是真實飛控。** 產生腳本是 `scratchpad/mission_accept_test.py:45-49`，
   起的是 `sitl/ArduPlane.exe --model quadplane`，是 SITL。
   pptx 自己也寫「SITL 實測」。報告寫「真的 ArduPilot 飛控」要改口。
2. **13 這個數字在本分支重現不出來。** 用該腳本的夾具離線重跑，
   `generate_waypoint_lines` 回 19 行 = 18 個任務項目，不是 13。
3. **航點數有兩個口徑，不要互相驗證。**
   `.waypoints` 檔是 1 行標頭 + **16** 列（index 0..15），
   briefing 說的「15 原始航點」是規劃器層的原始航點數，
   匯出時會再加上 `DO_SET_HOME`。

**其他今日親自量到的數字：**

* 20 個相關測試檔：96 failed / 415 passed / 2 collect errors，14.08 s。
* `tests/hv_dccpp/`（vendor 自帶）：832 passed。
  ⚠ `docs/PROJECT_STATUS.md:224` 寫「約 72 s」，實測 **559–654 s**，高估效能約 8–9 倍。
* `exports/vtol_recon_run2` 三架航點的合併包絡：南北 480 m × 東西 428 m，
  home 在 `25.077923 / 121.231859`（桃園）。

---

## 9. 環境與重跑

```bash
cd /c/Users/joy46/Documents/Projects/aeroplan-studio

# 開 GUI（左側分頁：基本演算法 / DCCPP / 戰術模組 / 蜂群打擊 / VTOL 偵察 / SITL）
python main.py

# 面板本身與相機/覆蓋模組的離線測試（今日 131 passed，約 3 s）
python -m pytest tests/test_vtol_recon_panel.py tests/test_recon_coverage.py -q -o addopts=""

# 無頭端到端（走產品路徑：get_params → plan_coverage_dccpp → verify_coverage）
python scratchpad/acceptance_vtol_recon.py

# SITL 階段驗收（起 3 台，讀回參數再試 ARM）
python scratchpad/acceptance_vtol_sitl.py

# 突變測試（一定要透過 bite_lock；先確認對照組是綠的）
python scratchpad/bite_fov_cone.py        # 今天唯一安全可跑的一支
```

⚠ `pytest.ini` 的 `addopts` 含 `-v`，會蓋掉命令列的 `-q`。要簡潔輸出就加 `-o addopts=""`。
⚠ `scratchpad/` 底下的驗收與突變腳本**全部未被 git 追蹤**。做這批工作時順手納管。

---

## 附錄 A　檔案地圖

**操作頁面與接線**
`ui/widgets/vtol_recon_panel.py`（面板本體，19 個信號）·
`ui/main_window.py`（掛載點與全部 handler）·
`ui/widgets/parameter_panel.py`（分頁容器）· `ui/widgets/dual_map_widget.py`（圈區）·
`ui/workers/planning_worker.py`（規劃丟背景執行緒）

**演算法**
`core/recon/sensor_footprint.py`（相機換算）· `core/recon/coverage_verifier.py`（覆蓋驗算）·
`core/dccpp/area_processor.py`（條帶生成）· `core/dccpp/task_allocator.py`（分配）·
`core/dccpp/idp_solver.py`（序列求解）· `core/dccpp/dccpp_path_builder.py`（Dubins 串接＋VTOL 剖面）·
`core/dccpp/vtol_profile.py`（剖面常數）· `core/dccpp/collision_avoidance.py`（避撞）·
`core/geometry/polygon.py` · `core/trajectory/dubins_trajectory.py`

**整合**
`mission/swarm_coordinator.py`（規劃唯一接縫）· `mission/vtol_mission_exporter.py`（匯出）·
`mission/sitl_launcher.py`（起 SITL）· `mission/sitl_link.py`（MAVLink 與解鎖）·
`utils/file_io.py`（QGC WPL 落字）

**產物**
`exports/vtol_recon_run1/`（修正前）· `exports/vtol_recon_run2/`（修正後）·
`exports/vtol_progress_report_2026-08-25.pptx` · `data/logs/UAVPathPlanner_20260825.log`

**文件**
`docs/design_2026-09-05_heterogeneous_vtol_planning.md`（驗證紀律）·
`docs/handoff_2026-08-25_auto_mission.md`（突變驗證最完整紀錄）·
`docs/adr/0007`～`0014`（HARVEST 側決策）· `docs/hv_dccpp/交接說明.md`（HARVEST 入口）

**HARVEST（另一套，別混用）**
`hv_dccpp/`（24 模組）· `hv_dccpp/integration/aeroplan_adapter.py`（轉接層）·
`scripts/hv_sitl_run.py`（無頭全鏈）· `runs/artifacts/HV4SITL2`～`13`

---

## 附錄 B　原報告中查不到出處的數字（本文件不採用）

依使用者指示「只留查得到的」。以下九項全 repo grep 只命中報告草稿
`docs/vtol_form_research_2026-09-03.md` 自己，沒有任何程式、測試或產物佐證，
因此不寫進正文。列在這裡是為了讓接手者知道它們**被刻意拿掉了**，
而不是被遺漏；要用就必須先重跑產生證據。

| 數字 | 報告中的說法 | 狀況 |
|---|---|---|
| 38.7 m | 測試區有一條 38.7 m 寬從未被掃到 | 無出處。`tests/test_dccpp_strip_pitch.py` 只寫「最寬達 `w_i/2` 的漏掃帶」，對面板預設 138.6 是 69.3 m，對不上 |
| 37.4 m | 邊緣 37.4 m 漏掃帶消失 | 無出處。全樹的 37.4 都是別的量（航點通過誤差、傾斜角） |
| 100.0% | 覆蓋率 97.0% → 100.0% | 前半有出處（見 §8），**後半沒有**。全樹覆蓋驗算紀錄裡沒有任何一筆是 2.0×1.5 km 的 100% |
| 2245.9 / 1897.0 | 全隊完成時間，快 15.5% | 無出處，且產生它所需的 `joint` 求解器不在樹上 |
| 187.2 / 145.6 s | 規劃時間 | 無出處 |
| 439 / 420 條 | 測試共 439 條、420 條離線全過 | 無出處，且**今天是假的**：實測 96 failed、兩個檔收集 0 項 |
| 121.6 m | 航次最高高度 | 無出處（`data/logs/` 與 `runs/artifacts/` 都沒有這趟的紀錄） |
| 0.46 m/s | 落地速度 | 無出處 |
| 1.1 km × 0.9 km | 桃園作業區尺寸 | 無出處。`exports/vtol_recon_run2` 三架航點的合併包絡實測只有 480 m × 428 m |

另外兩處**措辭**需要修正（不是數字問題）：

* 「重疊率怎麼調結果都一樣」→ 應為「中線間距不變，但條帶數量反向遞減，
  調高重疊率反而讓覆蓋率下降」。見 §5 缺陷①。
* 「上傳到真的 ArduPilot 飛控」→ 應為「上傳到 ArduPlane SITL（`--model quadplane`）」。見 §8。

---

*本文件由 Claude Opus 5 於 2026-09-09 產生。落地方式：7 個面向 × (1 落地 + 1 對抗性複查) = 14 個代理，共 996 次工具呼叫；所有測試數字由主線親自重跑確認。*
