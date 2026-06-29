# AeroPlan Studio 全專案煙霧測試報告

- **日期**: 2026-06-10
- **分支**: `refactor/strike-arch-rework-2026`
- **環境**: Python 3.14.3 / Windows 11 / pytest 9.0.2 / ruff 0.15.16 / vulture 2.16
- **範圍**: `config core mission sensors sitl ui utils cli scripts tests` + 根目錄腳本，共 **181 個 .py 檔（約 3.4 MB）**，排除 `.claude/worktrees`、`build`、`dist`、`__pycache__`

---

## 1. 總覽

| 檢查項目 | 工具 | 結果 |
|---|---|---|
| 語法編譯檢查 | `python -m compileall` | ✅ **通過**（0 語法錯誤） |
| Import 煙霧測試（逐模組匯入） | `scripts/_smoke_import_all.py` | ✅ **166/166 通過**（刻意跳過 2 個） |
| 單元測試 | `pytest tests/` | ✅ **149 passed**（0.89s） |
| 未定義名稱（潛在執行期錯誤） | ruff F821 | ⚠️ **8 處**（含 1 個確認的執行期 bug） |
| 重複定義 | ruff F811 | ⚠️ 2 處 |
| 未使用 import | ruff F401 | ⚠️ **205 處**（不含 tests） |
| 未使用區域變數 | ruff F841 | ⚠️ 27 處 |
| 無佔位符的 f-string | ruff F541 | ⚠️ 46 處 |
| 死代碼（信心 ≥80%） | vulture | ⚠️ 33 處（含 1 段 ~207 行不可達代碼） |

**結論**：專案整體健康——所有模組可正常匯入、語法無誤、測試全綠。但 lint 掃出 **1 個確定的執行期 bug（P0）**、數個斷裂的型別 forward reference，以及大量重構遺留的未使用 import / 死代碼（與 `docs/refactor_audit_2026.md` 的發現一致）。

---

## 2. 確認的 Bug（依嚴重度排序）

### 🔴 P0 — `mode` 未定義 → VTOL 蜂群打擊規劃完成時必炸 NameError

`ui/controllers/strike_controller.py:1269`（`_on_strike_execute_vtol` 內）：

```python
summary = (f'VTOL 蜂群打擊規劃完成：{len(vtol_plans)} 架 UCAV → '
           f'{mode} 發射 / {timing_mode} 時間協同')   # ← mode 未定義
```

該函式在 1155 行定義的是 `launch_mode`（2026 重構時改名），f-string 是執行期求值，
**只要 VTOL 打擊規劃成功走到結果摘要這行就會 NameError**。
修法：`{mode}` → `{launch_mode}`。
（pytest 沒抓到是因為 UI controller 層目前無測試覆蓋。）

### 🟠 P1 — `BuiltPath` forward reference 斷裂（5 處）

`core/dccpp/collision_avoidance.py:174, 276, 310, 377, 444`：

型別註解使用 `"BuiltPath"` 字串 forward reference，但整個模組**從未匯入** `BuiltPath`。
更可疑的是 314 行：

```python
from core.dccpp.dccpp_path_builder import BuiltWaypoint, SegmentLabel  # 兩者皆未使用
```

匯入了用不到的 `BuiltWaypoint`/`SegmentLabel`，卻沒匯入真正需要的 `BuiltPath`——
研判是重構時改錯了 import 目標。執行期因字串註解延遲求值不會炸，
但任何型別檢查器（mypy/pyright）與 `typing.get_type_hints()` 都會失敗。
修法：頂部加 `from typing import TYPE_CHECKING` + `if TYPE_CHECKING: from core.dccpp.dccpp_path_builder import BuiltPath`，並刪除 314 行的無用 import。

### 🟠 P1 — `WaypointSequence` forward reference 斷裂（2 處）

`mission/coverage_path.py:133, 209`：

`to_waypoint_sequence()` 回傳型別註解 `'WaypointSequence'`，但模組未匯入該類別
（應來自 `mission.waypoint`）。同上屬型別層級問題，修法相同（TYPE_CHECKING 匯入）。

### 🟡 P2 — `_angular_diff` 重複定義（shadow 掉重構後的共用版本）

`core/strike/swarm_strike_planner.py:603`：

59–68 行已依 2026 重構從 `core.strike.geometry` 匯入 `angular_diff as _angular_diff`
（檔內註解明言「為消除重複已遷移至 geometry 模組」），但 603 行仍留著舊的本地定義，
**模組載入後本地版本會覆蓋匯入版本**。兩者目前演算法相同所以行為無差，
但屬於重構未清乾淨的重複代碼，未來改 geometry 版本時此檔不會跟著變。
修法：刪除 603–606 行的本地定義。

### 🟡 P2 — `_on_strike_execute` 內約 207 行不可達死代碼

`ui/controllers/strike_controller.py:205`：

205 行 `return`（註解：「不再 fallthrough 到舊 TerminalStrikePlanner」）之後，
**207–413 行整段舊固定翼打擊路徑永遠不會執行**（vulture 100% 信心確認）。
這是該檔最大的一塊多餘代碼，建議整段刪除（git 歷史可隨時找回）。

### 🟡 P2 — `List` 於檔案中段重複匯入

`core/strike/geometry.py:334`：`from typing import Iterable, List` 與 24 行的 `List` 匯入重複。
修法：把 `Iterable` 併入頂部 import，刪除 334 行。

---

## 3. Import 煙霧測試明細

逐一 `importlib.import_module()` 全部 166 個模組（`QT_QPA_PLATFORM=offscreen`）：

- ✅ **166 OK / 0 FAIL**——無循環匯入、無缺失相依、無模組層級副作用錯誤
- ⏭️ 跳過 2 個：`sitl/sim_vehicle.py`（ArduPilot 外部工具，頂層即執行）、測試腳本自身
- 測試腳本保留於 `scripts/_smoke_import_all.py`，可隨時重跑：
  ```
  python scripts/_smoke_import_all.py
  ```

---

## 4. 多餘代碼統計

### 4.1 未使用 import（F401，共 205 處，不含 tests）

`ruff --fix` 可自動清除絕大多數。最集中的檔案：

| 檔案 | 數量 |
|---|---|
| `ui/dialogs/export_dialog.py` | 8 |
| `ui/main_window.py` | 8 |
| `mission/mission_manager.py` | 7 |
| `ui/widgets/servo_output_panel.py` | 7 |
| `ui/widgets/flight_logs_panel.py` | 5 |
| `core/global_planner/nfz_planner.py` | 5 |
| `core/strike/recon_to_strike_manager.py` | 5 |
| `core/strike/vtol_swarm_strike_planner.py` | 5 |
| `ui/widgets/polygon_editor.py` | 5 |
| `ui/widgets/tactical_swarm_strike_panel.py` | 5 |
| `mission/mavlink_exporter.py` | 5 |

常見模式：`typing.Optional/List/Tuple`（型別重構遺留）、`dataclasses.field`、
PyQt6 widget 類別（UI 改版遺留）、`numpy`（純 Python 化後遺留）。
另有 5 個檔案重複出現 `from core.strike.geometry import _angular_diff` 未使用
（`recon_to_strike_manager`、`advanced_recon_to_strike_manager`、`advanced_swarm_strike_planner`、`swarm_strike_planner`、`vtol_swarm_strike_planner`），同屬 geometry 重構遺留。

### 4.2 未使用區域變數（F841，27 處）

較值得處理的：

- `core/global_planner/coverage_planner.py` ×5（`circumference`、`center_lon`、`arc_end_lat/lon` 等——疑似演算法改版後的殘留計算）
- `core/strike/terminal_strike_planner.py:446` `rally_point`
- `core/vehicles/multirotor.py:247` `hover_time`
- `mission/swarm_coordinator.py:196` `loiter_time`
- `sensors/sensor_fusion.py:53` `covariance`

### 4.3 vulture 死代碼掃描（信心 ≥80%，33 處）

扣除誤報後的真實項目：

- ✅ 真死代碼：`strike_controller.py:209` 起的不可達段（見 P2）
- ✅ 真未使用：上述 F841 重疊項目
- ⚠️ **誤報（不要刪）**：
  - `cli/*_cmd.py` 的 `signum`——signal handler 固定簽名
  - `ui/widgets/{map_widget,cesium_map_widget,polygon_editor}.py` 的
    `lineNumber/sourceID/is_main_frame/nav_type`——Qt `javaScriptConsoleMessage`/`acceptNavigationRequest` 覆寫簽名，參數名不可省
  - `core/base/planner_base.py`、`grid_generator.py` 的 `occupancy_grid/grid_pos`——抽象介面參數

### 4.4 空 f-string（F541，46 處）

集中於 `core/global_planner/dccpp_heterogeneous_nfz_manager.py`（~15 處）與
`ui/widgets/map_widget.py`。純風格問題，`ruff --fix` 一鍵移除 `f` 前綴。

---

## 5. 單元測試結果

```
149 passed in 0.89s
```

| 測試檔 | 數量 |
|---|---|
| test_dccpp_pipeline.py | 46 |
| test_time_coordination.py | 20 |
| test_config_schemas.py | 16 |
| test_mission_tools.py | 16 |
| test_dtot_coordinator.py | 13 |
| test_xwing_tailsitter.py | 13 |
| test_dccpp_reroute_altitude.py | 7 |
| test_dccpp_coverage_retention.py | 6 |
| test_dccpp_return_leg_nfz.py | 6 |
| test_survey_return_leg_nfz.py | 6 |

**覆蓋缺口**：UI 層（`ui/controllers`、`ui/widgets`）完全無測試——P0 bug 正是藏在這裡。

---

## 6. 建議修復順序

1. **立即**：修 `strike_controller.py:1269` 的 `mode` → `launch_mode`（一行修掉執行期炸彈）
2. **本分支內**：補 `BuiltPath`、`WaypointSequence` 的 TYPE_CHECKING 匯入；刪 `_angular_diff` 舊本地定義；刪 `strike_controller.py` 207–413 死代碼段
3. **批次清理**：`python -m ruff check <dirs> --select F401,F541 --fix`（256 處可自動修），跑完重跑本煙霧測試 + pytest 驗證
4. **後續**：F841 逐項人工確認（部分變數可能是該用而未用的 bug 信號，例如 `coverage_planner.py` 的弧線終點座標）

## 7. 重跑指令

```powershell
python -m compileall -q config core mission sensors sitl ui utils cli scripts tests main.py
python scripts/_smoke_import_all.py
python -m ruff check config core mission sensors sitl ui utils cli scripts main.py --select F,E9
python -m vulture config core mission sensors ui utils cli main.py --min-confidence 80
python -m pytest tests/ -q
```
