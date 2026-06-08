# AeroPlan Studio 全專案重構盤點（2026）

> 目標：依 **DRY / KISS / SOLID** 評估整個專案可精簡與去重之處。
> 範圍：實際原始碼 **173 檔 / 約 82,000 行**（已排除 `.claude/worktrees`、`dist`、`build` 等暫存副本，以及 `sitl/sim_vehicle.py`、`sitl/vehicleinfo.py` 等 **vendored 上游 ArduPilot 檔——勿動**）。
> 狀態：**純分析盤點，尚未修改任何程式碼。** 本文件作為後續重構的路線圖。
> 分支：`refactor/strike-arch-rework-2026`

---

## 0. 一句話結論

可以顯著精簡。稽核找到**約 2,000–2,500 行純重複程式碼**可安全移除，外加數個 God Object 待拆分。全專案共通模式是：**「權威的單一來源其實已存在，但各模組都繞過它各寫一份」**——正是 DRY 要消滅的情況。其中**幾何數學的重複藏有真正的正確性 bug**（見 §1.1），不只是風格問題。

---

## 1. 五個跨系統關鍵發現

### 1.1 幾何數學被重寫 14–28 次，且含正確性 bug 〔DRY + 正確性〕🔴

`haversine` / `bearing` / 經緯度↔公尺投影 / Dubins 在全專案被獨立重寫了數十份，且**常數彼此不一致**：

| 來源 | 地球半徑 | 投影常數 |
|---|---|---|
| `utils/math_utils.py:86,158`（**應為權威來源**） | `6378137`(WGS84) | `111320·cosφ` |
| `core/geometry/coordinate.py:199,222` | `6378137` | — |
| `core/strike/geometry.py:37,48` | `6371000`(球面) | — |
| `core/global_planner/mdtsp_solver.py:105` / `fixed_wing_planner.py:230` / `coverage_planner.py:966` | `6371000` | `111111` |
| `mission/swarm_coordinator.py:582` / `mission/waypoint.py:163` | `6371000` / `6378137` | — |

**後果：同一條航線在不同模組算出的距離會差約 0.1%，是潛在的任務距離/時間計算 bug。**

重複的 haversine/bearing 至少 9–14 份；經緯度↔公尺投影至少 12 份（`111111` vs `111320` 分歧）；point-in-polygon 5 份；RDP 4 份；Dubins 4+ 份（`core/strike/geometry.py` 內部甚至有兩份：length solver `:145` 與 sampling `:365`）。

**處置**：全部收斂到 `utils/math_utils`（統一地球半徑為 WGS84 `6378137`，補一個 `haversine_3d`），其餘模組改為 import 或 re-export 以維持相容。
**可移除：全專案約 350–420 行**（單 haversine/bearing 群集），加上其他幾何群集合計可逼近 **1,100–1,400 行**。

### 1.2 `mdtsp_solver.py` 內含 ~390 行「預設不執行」的舊碼 〔DRY〕🔴

`core/global_planner/mdtsp_solver.py` 把 DCCPP 論文演算法重寫一份：`GreedyAllocator(:268)`、`IDPSolver(:420)`、`AltitudePlanner(:657)`——但 `DCCPPSolver.__init__:874-892` 在 runtime 會優先 import `core/dccpp/*` 的新版，舊版只在 `_use_new_dccpp=False` 時 fallback。

**處置**：刪除檔內 legacy 三件組，只依賴 `core.dccpp.*`（真正的 DIP），保留 dataclass。
**單檔可刪約 350–400 行，低風險。此為最大單點快贏。**

### 1.3 4 個地圖元件無共同介面，`DualMapWidget` 手抄 105 個轉發 〔DRY + DIP/LSP〕

`map_widget.py:103` / `cesium_map_widget.py:116` / `dual_map_widget.py:53` / `tkinter_map_widget.py:26` 各自重新宣告同一組 signal（`corner_added`/`circle_defined`/`nfz_polygon_drawn`/`strike_target_added`…）與方法名（`add_corner`/`clear_paths`/`set_edit_mode`/`update_uav_position`…）。`DualMapWidget` 對 `map_2d`/`map_3d` 手寫了 105 個一行轉發（`:242-259, 564-870`）。`MainWindow` 因此用 `hasattr()` 鴨子型別判斷（`main_window.py:448,484`）。

**處置**：建 `ui/widgets/map_base.py` 的 `MapWidgetBase(ABC)`，signal 與抽象方法只宣告一次；`DualMapWidget` 改用泛型 `_active()` 委派。
**可移除 200–350 行。** 另：`tkinter_map_widget.py` 為**死碼**（全專案無外部引用），可刪約 280 行。

### 1.4 兩個 main window 並存 〔SRP + 維護風險〕

`ui/main_window.py`（`MainWindow`，6001 行 God Object，含全部業務邏輯）與 `ui/aeroplan_main_window.py`（`AeroPlanMainWindow`，918 行，MIL-STD 新殼，僅 layout + 遙測 demo，**無任何任務規劃/打擊/SITL 邏輯**）。`main.py:235-240` 以 `--ui legacy|milstd` 並存。

兩者**不是程式碼複製，但重複了「狀態列 / 遙測讀數 / ARM / 主警告」這些 chrome 概念**，會各自漂移。`AeroPlanMainWindow` 顯然是目標架構（小元件組合 + `add_nav_page()` API）；`MainWindow` 應逐步拆成它託管的頁面。`aeroplan_main_window.py:830-909` 的 `_demo()` 區塊應移到 `examples/` 或測試。

### 1.5 Geofence 政策未落實——合規缺口，非重複 〔policy〕🔴

專案規則：「每個路徑產生器都必須透過 `GeofenceConstraintManager` 綁定 4 頂點矩形 Geofence」。實測：

- `core/strike/` 全 6 個產生器：grep `Geofence` **零命中**。
- `core/global_planner/` 的 coverage / fixed-wing / DCCPP / NFZ planner：**皆未呼叫** `GeofenceConstraintManager`。

好消息：manager 只有一個（`mission/geofence_manager.py:141`，單一來源乾淨），所以這是「在一個地方補上呼叫」而非去重。建議在共用的 `export_missions_qgc` 內統一綁定，讓所有 planner 一次符合。

另：`mission/geofence_manager.py:377-412`（硬寫 `FENCE_TYPE=7`）與 `mission/fence_zone.py:226-255`（動態計算同一 bitmask）**各自重寫了 ArduPilot FENCE_TYPE 位元遮罩**，且 `geofence_manager.Geofence` 與 `fence_zone.FenceZone` 是兩套無法互轉的模型——應收斂成一個。

---

## 2. God Objects〔KISS / SRP〕

| 檔案 | 行數 | 問題 | 建議拆分 |
|---|---|---|---|
| [main_window.py](../ui/main_window.py) | 6001 | 混 8 種職責 | `PathPlanningController`(`on_preview_paths:1370` 386 行)、`SitlController`(`on_sitl_*:5083-5751`)、`FenceController`(`on_fence_*`/`_apply_nfz_correction_*`)、`ExportService`、`TacticalOverlayController`。Strike 已先拆成 `StrikeControllerMixin`——照此模式。 |
| [parameter_panel.py](../ui/widgets/parameter_panel.py) | 4461 | 涵蓋覆蓋/NFZ/高程切片/SAR/雷達/打擊命令 | 各 `_create_*_panel`(`:2542,2608,2699,2839`) → 獨立子 widget（同時收掉 86 處 inline `setStyleSheet`） |
| [strike_controller.py](../ui/controllers/strike_controller.py) | 2150 | DTOT/STOT/VTOL/recon/碰撞/視覺化/5 種匯出混一起 | `StrikePlanController` / `StrikeExportService` / `StrikeVizController` |
| [swarm_coordinator.py](../mission/swarm_coordinator.py) | 1545 | `plan_coverage_dccpp:1023` 單一方法 380 行 | DCCPP 編排移入 `core/dccpp/DccppPipeline`；`__main__` demo 移到 `tests/` |
| [sitl_link.py](../mission/sitl_link.py) | 1118 | transport+遙測解碼+上傳協定+VTOL 邏輯混一起 | `MavlinkTransport` / `TelemetryDecoder` / `MissionUploader`+`FenceUploader` |
| [sitl_launcher.py](../mission/sitl_launcher.py) | 739 | Windows 防火牆 + 140 行 VTOL `.parm` heredoc + 程序生成 | `WindowsFirewall`/`PortScanner` util；VTOL parm 移到 `sitl/default_params/` 資料檔 |
| [coverage_planner.py](../core/global_planner/coverage_planner.py) | 1432 | grid/spiral/circle 生成 + 幾何工具 + 兩個 QGC 寫檔器混一起 | `ScanPatternStrategy` 子類；幾何 statics 移到 `core/geometry`；寫檔器抽出 |
| [diamond_swarm_planner.py](../core/strike/diamond_swarm_planner.py) | 890 | `plan():321-633` 把腿長迴圈**算了兩次**(`:459-503` 與 `:532-631`) | 合併為單次 `_compute_legs()`，同時修掉漂移 bug |

---

## 3. 各子系統詳細發現

### 3.1 `core/strike/`（打擊規劃——本次重構主場）

2026 重構已正確抽出 `geometry.py` 與 `time_coordination.py`，但**抽取只做一半**：managers 仍保留私有重寫，且第二層重複（slot 指派、speed/loiter 決策、MissionItem→QGC 匯出、mission 序列組裝、briefing 寫檔）從未處理。

**主要重複群集（可移除 ~600–650 行）：**
1. **`MissionItem`→QGC WPL 110 匯出迴圈**：`swarm:307` / `advanced_swarm:302` / `vtol:421` / `recon:824` / `diamond:830` 近乎逐字複製 5 份 → 抽 `core/strike/mission_export.py::export_missions_qgc`（~90 行）。
2. **`_solve_speed_or_loiter` 速度/盤旋決策**：`advanced_swarm:488` / `vtol:652` / `swarm:465` 三份結構相同，且 `time_coordination.compute_loiter_plan` **早已存在**只是沒人用 → 改呼叫它（~150 行）。
3. **`_assign_omnidirectional_slots`（360°向量+最小成本旋轉對齊）**：4 份（`swarm:349`/`advanced_swarm:332`/`vtol:449`/`recon:444`）→ `geometry.assign_omnidirectional_slots`（~120 行）。
4. **`_build_mission` MAVLink 序列組裝**：5 份共用同一骨架 → `MissionBuilder` helper（~80 行，中風險）。
5. **managers 內殘留的幾何重寫**：`recon_to_strike_manager.py:155-200` 的 `_latlon_to_enu`/`_enu_to_latlon`/`_vec3_*` 與 `geometry.*` **逐字相同**（`advanced_recon` 已正確 import）；`dtot_coordinator.py:130` 與 `terminal_strike_planner.py:94` 各有私有 `_haversine`/`_R_EARTH` → 刪除改 import（~110 行，**最低風險、最高確定性**）。

**SOLID：**
- **五個 swarm planner 無共同基底**，卻有穩定共同契約（`plan()→List[Plan]`、`export_qgc_wpl`、`_assign_*`、`_build_mission`、`plans`）→ 引入 `BaseSwarmStrikePlanner`（template method）。**最高槓桿的結構改動。**
- **以 `swarm_strike_planner` 當工具模組**：`advanced_swarm`/`vtol`/`recon`/`advanced_recon` 都 `from ...swarm_strike_planner import _haversine,_bearing_deg,...`——其實是 `geometry` 的 re-export。應直接 import `geometry`。**注意**：`ui/controllers/strike_controller.py:1201,1689` 依賴 `swarm_strike_planner` 的 `_destination`/`_haversine` re-export，是 load-bearing 公開 API，去耦時須一併重導或保留 wrapper。
- **三套平行 `FeasibilityStatus` enum**（`swarm:121`/`advanced_swarm:58`/`vtol:187` + `time_coordination:99`）→ 統一 alias 到 `time_coordination` 的 superset。

### 3.2 `core/global_planner` / `core/dccpp` / `core/trajectory` / `core/geometry`

- 幾何/QGC 重複見 §1.1、§1.2。
- **QGC WPL 110 / MAVLink waypoint-line 產生**：~11 處（`coverage_planner.py:775,852` 兩個近乎相同的 76–78 行寫檔器、`fixed_wing_planner.py:1042` 121 行、`heterogeneous_nfz_planner.py:63`、6 個 strike planner）→ `WaypointFileWriter`/`wpl_line()`（~150–200 行，動到輸出格式，需逐 byte 驗證）。
- **長方法**：`DubinsTrajectoryGenerator._compute_csc`(160 行，全稽核最長)、`heterogeneous_nfz_planner.plan`(127)、`fixed_wing_planner`(121/118/92)、`path_cost_calculator.compute_transition_matrix`(108)。
- **兩個 NFZ planner 的 `_apply_fillets`**（`dccpp_heterogeneous_nfz_manager.py:205` vs `heterogeneous_nfz_planner.py:482`）漂移成近重複 → 提到共用 `FixedWingDubinsPlanner` 基底。
- 好榜樣：`path_cost_calculator.py:422 _compute_exact_dubins` 正確委派給 `DubinsTrajectoryGenerator`——這就是要複製的模式。

### 3.3 `ui/`

- 見 §1.3（地圖元件）、§1.4（雙 main window）、§2（God Objects）。
- **inline 樣式違反 MIL-STD QSS 政策**：`Global_MIL_STD.qss` 已提供 `[role=]`/`[msgSeverity=]` 等 property selector，但被大量 inline 字串繞過——`parameter_panel.py`(86 處 setStyleSheet + 87 硬編 hex)、`sitl_hud.py`(29+26)、`dual_map_widget.py`(`_btn_qss/_btn_style:24-45`)、`strike_ttt_dashboard.py`(29)、`fleet_card.py`(17)。**諷刺的是連 MIL-STD 參考視窗自己也違規**：`aeroplan_main_window.py:389-392,458-460` 用 f-string `color:`/`border:` 設 ARM/SAFE/FAIL → 改 `setProperty('armState',...)` + QSS。
  - *合理例外（保留 inline）*：`map_widget.py` 的 94 個 hex 多為 Folium/Leaflet **JS 地圖圖層顏色**，非 Qt widget 樣式，QSS 無法處理。
- **共用 helper 機會**：`repolish(widget)`（`unpolish`/`polish` 樣板重複 ~10 處）、`set_status(label,msg,severity)`、`run_modal(dialog_cls)`、`_add_action(toolbar,...)`（`main_window.py:647-692` 7 次 QAction 樣板）。

### 3.4 `mission/` / `sitl/` / `cli/` / `config/`

- 幾何重複見 §1.1；geofence 雙模型見 §1.5。
- **SITL fence 上傳序列在 `sitl_link.py` 內重複 3 次**：`_send_fence:786` / `_send_fence_zones:865` / `_send_mission:1042` 都重複同一 6 步協定（disable→clear→count→loop request/item→ack→param→enable）→ 抽 `_upload_mission_items(items, mission_type)`（~70–90 行）。
- **SITL 設定/連接埠公式重複**：`{"sysid":i+1,"embedded_port":14550+10*i}` 在 `cli/sitl_cmd.py:78` / `cli/demo_cmd.py:48` / `sitl_launch_dialog.py` 各寫一份；`5760+10*instance` TCP 埠公式在 `sitl_launcher.py:124,476,491,603,641` 散佈 → `SITLLauncher.tcp_port_for()` / `default_instance_configs()`。
- **param-set 重複耦合**：`param_service.py:92-119` 直接戳私有 `link._mav.mav.param_set_send`，繞過 `SITLLink.set_param`；force-arm(21196)+`ARMING_CHECK=0` 在 `:606,676,701` 複製 3 次。
- **設定散落**：`get_settings()` 全域單例被 8 檔 ad-hoc 取用；同時 `mission/` 又硬編同樣的值（home `23.7/121.0` 在 `sitl_launcher.py:351,437`、兩個 CLI、`MapSettings.default_lat` 重複）→ 注入 `Settings` 物件、`GlobalSettings` load/save 改 registry 驅動。
- **MAVLink transport 無抽象**：`MockSITLLink` 與真實 `SITLLink` 介面不一致（mock 有 `connect()`、缺 `telemetry` signal）→ 不是合格 LSP 替身 → 定義 `MavlinkTransport` Protocol。

---

## 4. 分階段執行計畫

### Phase 0 — 安全機械快贏（低風險、無 API 變動，約移除 1,200–1,600 行）

| # | 動作 | 檔案 | 移除行數 | 風險 |
|---|---|---|---|---|
| 0-1 | 刪 `mdtsp_solver.py` legacy 三件組，只依賴 `core.dccpp.*` | 1 | 350–400 | 低–中 |
| 0-2 | 全專案幾何函式統一走 `utils/math_utils`（**修地球半徑 bug**），其餘 re-export | ~10 | 180–220 | 低 |
| 0-3 | 刪 `recon_to_strike_manager` 的 ENU/vec3 拷貝，改 import `geometry` | 1 | ~50 | 低 |
| 0-4 | 抽 `core/strike/mission_export.export_missions_qgc` + 移 `MissionItem`，重接 5 匯出器 | 6 | ~90 | 低 |
| 0-5 | 加 `geometry.assign_omnidirectional_slots`，收掉 4 份 | 5 | ~120 | 低 |
| 0-6 | 移 `terminal_strike_planner`/`dtot_coordinator` 私有 `_haversine`/`_R_EARTH` 到 `geometry` | 2 | ~60 | 低 |
| 0-7 | 刪死碼 `tkinter_map_widget.py` | 1 | ~280 | 低 |
| 0-8 | 加 `SITLLauncher.tcp_port_for()`/`default_instance_configs()`，CLI 與 dialog 共用 | 4 | ~25 | 低 |

### Phase 1 — 結構去重（中風險，須先補測試）

| # | 動作 | 檔案 | 影響 | 風險 |
|---|---|---|---|---|
| 1-1 | `MapWidgetBase` ABC + `DualMapWidget` 改泛型委派（收 105 轉發） | 5 | −200~350 行 | 中 |
| 1-2 | 三個 swarm planner 改呼叫 `time_coordination.compute_loiter_plan` | 3 | −150 行 | 中（動數值，先測 STOT/DTOT） |
| 1-3 | `diamond` 雙腿長迴圈合一 `_compute_legs()` | 1 | −45 行 + 修 bug | 中 |
| 1-4 | 統一 fence FENCE_TYPE bitmask builder；`param_service` 改呼叫 `link.set_param` | 4 | −40 行 | 中 |
| 1-5 | `sitl_link` 抽 `_upload_mission_items()`，收 3 份上傳迴圈 | 1 | −70~90 行 | 中（live MAVLink timing） |
| 1-6 | inline 靜態樣式移回 `Global_MIL_STD.qss`（先 `sitl_hud`/`dual_map`/`parameter_panel`） | ~8+QSS | −150~200 行 | 低 |
| 1-7 | 加共用 helper：`repolish()`/`set_status()`/`run_modal()`/`_add_action()` | 多 | −100~150 行 | 低 |
| 1-8 | 抽 `WaypointFileWriter`/`wpl_line()`，收 ~11 處 WPL 寫檔 | ~11 | −150~200 行 | 中（輸出格式，逐 byte 驗） |

### Phase 2 — 架構級（高風險、影響公開 API、須完整測試覆蓋）

| # | 動作 | 風險 | 破壞 API？ |
|---|---|---|---|
| 2-1 | 引入 `BaseSwarmStrikePlanner` template method，三 planner 改子類 | 高 | 可能 |
| 2-2 | 拆 `MainWindow` God Object → `PathPlanningController`/`SitlController`/`FenceController`/`ExportService` | 高 | 否（內部） |
| 2-3 | 收斂雙 main window：`AeroPlanMainWindow` 為唯一殼，`MainWindow` 拆成託管頁面 | 高 | 是（最終移除 `--ui legacy`） |
| 2-4 | `MavlinkTransport` Protocol，`MockSITLLink` 成合格替身 | 高 | 可能 |
| 2-5 | `Geofence`↔`FenceZone` 模型合一；兩條上傳路徑只吃 `FenceZone` | 高 | 是 |
| 2-6 | **補上 geofence 政策落實**（合規，非去重）：在共用匯出綁定 4 頂點矩形 fence | 中 | 否 |
| 2-7 | 拆 `swarm_coordinator.plan_coverage_dccpp` → `core/dccpp/DccppPipeline` | 中 | 否（方法保留為 facade） |

---

## 5. 執行順序建議

1. **先 Phase 0**：純死碼/重複對既有權威來源去重，幾乎零 API 風險，立即 −1,200~1,600 行。**0-2 同時修掉地球半徑正確性 bug，價值最高。**
2. **Phase 1 前先補測試**：尤其 STOT/DTOT timing、diamond 腿長、WPL 輸出格式、fence 上傳——這些動到數值或對外格式。
3. **Phase 2 視需求**：架構級重構，建議搭配 ADR（`docs/adr/`）記錄決策後再動。

> 註：本盤點為 2026-06-08 程式碼快照；行號可能隨後續修改漂移，動工前請以實際檔案為準。
