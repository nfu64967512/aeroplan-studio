# hv_dccpp → AeroPlan Studio 接入手冊

**版本** 1.0（2026-09-06）　**對象** 要把 `hv_dccpp` 接進 AeroPlan Studio v2.8.1（PyQt6 地面站）的工程師
**轉接層** `hv_dccpp/integration/aeroplan_adapter.py`（1858 行）　**測試** `tests/test_aeroplan_adapter.py`（48 項；全套 809 項全過，105.77 s）

本文中：`REPO/x/y.py:NNN` = AeroPlan Studio 儲存庫（<https://github.com/nfu64967512/aeroplan-studio>）之檔案與**已驗證行號**；
`hv_dccpp.<模組>.<函式>` = 本專案（`C:/Users/joy46/OneDrive/文件/vtol演篹法`）。
所有數字皆為本機實測（Python 3.13、Windows 10），平台數值一律引用 `docs/平台參數表.md`（權威來源，**不可更改**）。

---

## 0. 這份手冊是什麼

`hv_dccpp` 是一套已完成的異構雙構型 VTOL 群飛覆蓋路徑規劃函式庫。AeroPlan Studio 內部已有一套依同一篇論文
（Wang et al. 2025, *Defence Technology* 47:197-215）實作的 DCCPP。本手冊說明**如何把前者接進後者，且不需修改任何下游程式**。

核心承諾只有一句：

> `HvDccppCoordinator.plan_coverage_dccpp(...)` 的簽章與 `REPO/mission/swarm_coordinator.py:975` **逐字相同**，
> 回傳同樣的 6 鍵字典（`REPO/mission/swarm_coordinator.py:1291-1298`）。

`test_signature_matches_swarm_coordinator` 以 `inspect.signature` 逐一釘住 15 個參數名、順序與 12 個預設值。

### 30 秒摘要：你想做什麼 → 讀哪一節

| 你想做什麼 | 節 |
|---|---|
| 看它比既有 DCCPP 多了什麼、少了什麼 | §1 能力對照表 |
| 把套件裝進 AeroPlan 的 venv、PyInstaller | §2 |
| 「現在就想跑一次」（不改 AeroPlan 任何檔案） | §3 五分鐘上手（腳本 + 真實輸出） |
| 資料從哪來、到哪去 | §4 資料流 |
| **最小改動接入（推薦）** | §5 方式 A（`swarm_coordinator.py:975` 內切換） |
| 兩規劃器並存、A/B 比較 | §6 方式 B（新分頁 + 新 signal） |
| **UI 不要凍結（必做）** | §7 `ui/workers/planning_worker.py` |
| UI 的 25 個參數各自去哪了 | §8 |
| 座標／航向／高度基準對不上 | §9 |
| 兩構型參數怎麼填、`class_of` 怎麼寫 | §10 |
| 地形／禁飛區／Home／備降點 | §11 |
| 匯出 `.waypoints`、上傳 SITL | §12 |
| 飛行中重規劃 | §13 |
| SITL 驗證 | §14 |
| 出錯了 | §15 錯誤對照表、§17 FAQ |
| 慢／記憶體／限制 | §16 |
| 之後要補什麼 | §18 |
| API 速查 / 接入清單 | 附錄 A / 附錄 B |

---

## 1. hv_dccpp 是什麼、與既有 core/dccpp 的關係

### 1.1 同一篇論文，兩種實作深度

AeroPlan 既有 DCCPP 是論文的忠實實作：Alg.1 配置 `REPO/core/dccpp/task_allocator.py:181`、
Alg.2 IDP `REPO/core/dccpp/idp_solver.py:123`、Alg.3 高度 `REPO/core/dccpp/altitude_planner.py:96`、
§2.2/2.3.1 掃描線 `REPO/core/dccpp/area_processor.py:345`、Dubins 組裝 `REPO/core/dccpp/dccpp_path_builder.py:94`、
防撞 `REPO/core/dccpp/collision_avoidance.py:444`。

`hv_dccpp` 以同一框架為基礎，往「**特定兩台真實機**（Pioneer PNP / HERO PNP，皆為 lift-and-cruise 4+1 複合翼 VTOL）
飛得起來的完整任務」擴展：感測器物理、能量、風、地形高度包絡、起降走廊、4-D 去衝突、ArduPilot QuadPlane 輸出。

### 1.2 能力對照表

| 能力 | AeroPlan 既有 `core/dccpp` | `hv_dccpp` |
|---|---|---|
| **異構雙構型** | `UAVState.turn_radius`/`vehicle_type` 逐機有效（`task_allocator.py:136-142` 依型別選 Dubins/haversine），但 UI 用**同一組 spinbox** 產生所有機（`REPO/ui/main_window.py:4722-4744`）、IDP 與 Builder 共用單一 `R_min`（`REPO/mission/swarm_coordinator.py:1184` `_R = max(u.turn_radius ...)`） | 構型 A（Pioneer + 5 kg LiDAR）／B（HERO + 0.6 kg 相機）逐機獨立：轉彎半徑、巡航空速、爬升／下降角、續航、最低固定翼作業高度（`hv_dccpp.platform.PlatformSpec`） |
| **感測器驅動刈幅** | 🔴 `fov_width` **寫入但全樹無人讀**（`REPO/mission/swarm_coordinator.py:1090`）；區域分解只用單一 `coverage_width_m`（`:1187-1195`） | 刈幅由感測器硬體 + 作業高度推導（`LidarHardware`/`CameraHardware`/`TrapezoidFovSpec`）。實測 160 m AGL：A = 224 m、B = 235 m。UI 的 `coverage_width_m` 只作核對，差 > 25% 記 `fov_width_mismatch` |
| **能量與返航** | 🔴 完全沒有：`REPO/config/vehicle_profiles.yaml` 有 `max_flight_time_min` 但無人讀 | 逐機 `SortieEnergy`（AUW、酬載、可用 Wh）+ `EnergyModel`；檢查「作業 + 轉場 + RTB + 保留」能否閉合（表 1 `reserve_fraction` 0.30） |
| **風** | 🔴 無 | 高度分層 `WindLayer`；修正地速、耗時與**轉彎半徑** `spec.r_eff(W)`（實測 A 200.0 → 244.7 m @8 m/s；B 120.0 → 151.5 m）、逆風進場（D19） |
| **坡度受限高度包絡** | `AltitudePlanner` 存在，但 🟠 **live 路徑從未呼叫它**（`REPO/mission/swarm_coordinator.py:1177-1298`）；legacy 路徑呼叫 `to_latlon_list()` 而 live `BuiltPath` 無此方法 | 一律做（規格 §9、Alg.3）：以**自然地形**量 AGL，受 `planning_climb_angle_deg` 3.0° / `planning_descent_angle_deg` 4.0°（表 1）限制，逐條帶分層 |
| **起降走廊** | 🟠 只有 UI 的跑道方位／長度（VTOL 用不上） | 垂直起降走廊 + MC↔FW 轉換 + 走廊排程（規格 §13.5），含 D46 錯開起飛（`dt_dep_s` 預設 60 s） |
| **動態事件** | `REPO/core/global_planner/mdtsp_solver.py:1018 solve_dynamic_event` 存在但未接 UI | 9 種事件型別；實測 `EnergyUpdate` 重規劃 **167.8 ms**（§13） |
| **4-D 去衝突** | `CollisionAvoidance` 為**後處理**，UI 勾選預設 False（`REPO/ui/main_window.py:4849`） | 時空分離為**規劃內建**（規格 §10.6、D14），關不掉。`d_sep_m` 100.0 m、`dh_sep_m` 50.0 m |
| **ArduPilot QuadPlane 輸出** | 有 `.waypoints`（`REPO/ui/main_window.py:5213`）與 VTOL 匯出（`REPO/mission/vtol_mission_exporter.py:219`）；🔴 起飛指令有 bug（§12.3） | `hv_dccpp.ardupilot.export_all` → 每機 `.waypoints` / `_rally.txt` / `_fence.txt` / `.param` / `.plan` + `mission_summary.json` |
| **座標基準** | 🟠 混用：`REPO/core/geometry/region_divider.py:260-283` 用 111111.0 m/deg，`CoordinateTransformer` 用橢球（`REPO/docs/refactor_audit_2026.md:18-35` 自承） | 全程 WGS84 橢球（`hv_dccpp.geo.GeoFrame`，經 ECEF）→ 與舊值差約 **0.1%** |

### 1.3 誠實面：既有實作有、hv_dccpp **尚未涵蓋**的

1. **障礙物**（`REPO/ui/main_window.py:152 self.obstacles`、`REPO/core/collision/obstacle_manager.py`）——但這些原本**也沒**接進 DCCPP。
   `hv_dccpp` 有 `Obstacle` 型別（D24 燒入地形），需呼叫端組裝；`to_mission_inputs` 目前一律傳 `obstacles=()`。
2. **多旋翼與純固定翼**：hv 只支援 lift-and-cruise 4+1 VTOL 兩構型。UI 選 `multirotor`/`fixed_wing` 時**不可**走 hv 分支（§5.3 的閘門）。
3. **legacy `DCCPPSolver` fallback**（`REPO/mission/swarm_coordinator.py:1299-1328`）：hv 分支沒有等價物，改以 `try/except ImportError` 降級。
4. **`CoordinatedPlanner.synchronize`**（`coordination_mode="coordinated"`）：hv 不提供（一律 4-D 去衝突，語意不同）。
5. **NFZ 重繞後處理**（`REPO/ui/main_window.py:4844`）：hv 規劃時已避開 `NoFlyZone`，這層變多餘（§5.4）。
6. **Rally 點**：AeroPlan 是空 stub（`REPO/mission/geofence_manager.py:467`）；hv 能產生但接 UI 需另做（§11.4）。
7. **地圖圖層／Cesium／HUD／專案存檔**：轉接層只換規劃器，完全不碰 UI。

---

## 2. 安裝

### 2.1 相依性：已全部滿足

`hv_dccpp/pyproject.toml` 宣告 `numpy>=1.26`、`scipy>=1.11`、`shapely>=2.0`、`matplotlib>=3.8`；
extras：`mavlink`（pymavlink）、`mqtt`（paho-mqtt）、`test`（pytest）。
AeroPlan 的 `REPO/requirements.txt` 已含 numpy/scipy/matplotlib/shapely/pymavlink≥2.4.37 → **不需安裝任何新第三方套件**
（`paho-mqtt`、`rasterio` 都不需要，見 §11.1）。

> ⚠ hv 的版本下限高於 AeroPlan（1.26 vs 1.24 等）。`pip install -e` 會自動升級；升級後請重跑 AeroPlan 自己的 `pytest`。

### 2.2 裝進 venv

```powershell
cd <AeroPlan 專案根>
.\.venv\Scripts\activate
pip install -e "C:/Users/joy46/OneDrive/文件/vtol演篹法"
# 需要 MAVLink 直連（§13.4、§14）時：
pip install -e "C:/Users/joy46/OneDrive/文件/vtol演篹法[mavlink]"
```

### 2.3 `REPO/requirements.txt`

```text
# HV-DCCPP 異構雙構型 VTOL 覆蓋規劃
-e file:///C:/Users/joy46/OneDrive/文件/vtol演篹法
# 或跨機重現：hv_dccpp @ git+https://<你的 git 主機>/hv_dccpp.git@v0.1.0
```

同時把 `numpy>=1.24.0`→`>=1.26.0`、`scipy>=1.10.0`→`>=1.11.0`、`matplotlib>=3.7.0`→`>=3.8.0`，避免解析出不相容組合。

### 2.4 PyInstaller

`REPO/aeroplan_studio.spec:138-143` 現況是一串 `collect_submodules('core'|'mission'|'ui'|'utils'|'config'|'sensors')`。
加一行——**必須用 `collect_submodules`**，因為 `hv_dccpp/__init__.py` 以 PEP 562 `__getattr__` 延遲載入整合層，靜態分析看不到：

```python
hiddenimports += collect_submodules('sensors')
hiddenimports += collect_submodules('hv_dccpp')      # ← 新增：延遲載入的整合層必須全收
```

`collect_data_files('hv_dccpp')` **目前不需要**——套件內沒有任何非 `.py` 資料檔（情境全由 `hv_dccpp/scenario.py` 以程式產生）。
日後若放了預設情境 JSON 才需要 `datas += collect_data_files('hv_dccpp')`。
`spec:145-164` 的 `excludes`（pytest/pandas/tkinter/PyQt5…）不影響 hv：本套件不用 pandas、不用 tkinter，
matplotlib 只在存取 `hv_dccpp.viz` 時才載入（§17.5）。

### 2.5 驗證安裝的三行指令

```powershell
python -c "import hv_dccpp as hv; print(hv.__version__)"
python -c "from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator as C; import inspect; print(list(inspect.signature(C.plan_coverage_dccpp).parameters)[1:])"
python -m pytest "C:/Users/joy46/OneDrive/文件/vtol演篹法/tests/test_aeroplan_adapter.py" -q
```

實測輸出：

```text
0.1.0
['areas', 'drones', 'coverage_width_m', 'overlap_rate', 'auto_scan_angle', 'coverage_altitude', 'terrain_func', 'enable_altitude', 'mounting_angle_deg', 'dem_path', 'coordination_mode', 'runway_bearing_deg', 'runway_length_m', 'landing_bearing_deg', 'landing_rollout_m']
48 passed in 4.71s
```

第二行的 15 個參數名與 `REPO/mission/swarm_coordinator.py:977-991` 逐字相同——這是整個接入的前提。
---

## 3. 五分鐘上手：不改 AeroPlan 任何檔案

腳本完全獨立，不 import AeroPlan、不需要 PyQt6。

```python
"""hv_quickstart.py — 用 HvDccppCoordinator 跑一次規劃並印出 AeroPlan 會拿到的 6 鍵結果。"""
import math, time
from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator

HOME = (23.95, 120.90)
M_LAT, M_LON = 111132.0, 111412.0 * math.cos(math.radians(23.95))
ll = lambda x_m, y_m: (HOME[0] + y_m / M_LAT, HOME[1] + x_m / M_LON)   # ENU(東,北) m → (lat, lon)

areas = [{"area_id": 1, "priority": 1.0,                              # 形狀同 main_window.py:4709
          "polygon": [ll(4000, -600), ll(7000, -600), ll(7000, 600), ll(4000, 600)]}]   # 3 km × 1.2 km
drones = [                                                            # 形狀同 main_window.py:4721-4744
    {"drone_id": 1, "name": "UAV-1", "position": HOME, "speed": 21.5, "heading": 0.0,
     "turn_radius": 200.0, "vehicle_type": "vtol", "fov_width": 160.0},
    {"drone_id": 2, "name": "UAV-2", "position": HOME, "speed": 19.0, "heading": 0.0,
     "turn_radius": 120.0, "vehicle_type": "vtol", "fov_width": 160.0}]

report: dict = {}
coord = HvDccppCoordinator(
    class_of=lambda d: "A" if int(d["drone_id"]) == 1 else "B",       # ★ 必填，見 §10.3
    progress=lambda f, m: print(f"  [{f * 100:5.1f}%] {m}"), on_report=report.update)

t0 = time.perf_counter()
res = coord.plan_coverage_dccpp(          # ← 簽章與 swarm_coordinator.py:975 逐字相同
    areas=areas, drones=drones, coverage_width_m=160.0, overlap_rate=0.25, coverage_altitude=160.0)
print(f"\n規劃耗時 {time.perf_counter() - t0:.2f} s  status={report['status']}")
print(f"total_makespan = {res['total_makespan']:.1f} s   allocation = {res['allocation']}")
for did, bp in sorted(res["assembled_paths"].items()):
    print(f"UAV-{did} 構型{bp.cls}({bp.hv_uav_id}) {len(bp.waypoints):3d} 航點  總長 "
          f"{bp.total_length_m / 1000:6.2f} km  落地 {bp.makespan_s:7.1f} s  "
          f"最高 {max(w.alt for w in bp.waypoints):6.1f} m")
    print(f"        起飛 {bp.takeoff_length_m:7.1f} + 進場 {bp.entry_length_m:7.1f} + 作業 "
          f"{bp.operation_length_m:8.1f} + 轉移 {bp.transfer_length_m:7.1f} + 降落 {bp.landing_length_m:7.1f} m")
for did, tl in sorted(coord.upload_tuples(vtol_transition_alt_m=100.0).items()):   # → sitl_link.py:264
    print(f"UAV-{did} 上傳 {len(tl)} 項，前 3 = {tl[:3]}")
for k, lines in sorted(coord.qgc_waypoints(key="drone_id").items()):               # QGC WPL 110
    print(f"UAV-{k} QGC WPL {len(lines)} 行；第 2 行 = {lines[1]!r}")
print("timing_ms =", report["timing_ms"])
```

### 真實輸出（本機實測 2026-09-06）

```text
  [  2.0%] 組裝任務輸入          [ 15.0%] 建立情境（地形／風場／能量模型）
  [ 35.0%] 前置檢查（preflight）  [ 40.0%] 規劃中（分解 → 配置 → IDP → 組裝 → 去衝突）
  [ 90.0%] 轉換為 AeroPlan 結果格式                              [100.0%] 完成

規劃耗時 1.20 s  status=GO_WITH_WARNINGS
total_makespan = 1479.4 s   allocation = {1: [1, 2]}
UAV-1 構型A(A1)  74 航點  總長  24.55 km  落地  1259.4 s  最高  169.9 m
        起飛   468.0 + 進場  4228.8 + 作業  12426.4 + 轉移  2631.5 + 降落  4791.8 m
UAV-2 構型B(B2)  44 航點  總長  22.79 km  落地  1479.4 s  最高  160.4 m
        起飛   384.0 + 進場  4193.3 + 作業   9344.5 + 轉移  4362.5 + 降落  4503.8 m
UAV-1 上傳 78 項，前 3 = [(0.0, 0.0, 0.0, 178, 0.0, 21.5, -1.0, 0.0), (23.95, 120.9, 100.0, 84, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 3000, 4.0, 0.0, 0.0, 0.0)]
UAV-2 上傳 48 項，前 3 = [(0.0, 0.0, 0.0, 178, 0.0, 19.0, -1.0, 0.0), (23.95, 120.9, 100.0, 84, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 3000, 4.0, 0.0, 0.0, 0.0)]
UAV-1 QGC WPL 86 行；第 2 行 = '0\t1\t0\t16\t0.0\t0.0\t0.0\t0.0\t23.950000\t120.900000\t0.00\t1'
UAV-2 QGC WPL 54 行；第 2 行 = '0\t1\t0\t16\t0.0\t0.0\t0.0\t0.0\t23.950000\t120.900000\t0.00\t1'
timing_ms = {'context': 0.0, 'preflight': 15.7, 'decompose': 222.1, 'layering': 284.7, 'corridors': 24.0,
             'cache': 100.4, 'allocate': 56.9, 'idp': 29.0, 'assemble_3d': 107.8, 'deconflict': 68.7, 'total': 909.3}
```

三個重點：(1) **`allocation = {1: [1, 2]}`** 表示兩構型都被分配到工作（A 4 條帶、B 3 條帶）；只有一台在飛
八成是 `PayloadRequirement` 沒涵蓋兩種感測 kind（§15.2 第 7 條）。(2) 上傳前 3 項＝`DO_CHANGE_SPEED(178)` →
`NAV_VTOL_TAKEOFF(84)@100 m` → `DO_VTOL_TRANSITION(3000, p1=4=FW)`，與 `REPO/ui/main_window.py:6117-6125` 一致
（但修掉了他們的起飛遺漏 bug，§12.3）。(3) **`GO_WITH_WARNINGS` 正常**：warnings 多半是 `provisional_params(...)`
與 `avionics_unvalidated(...)`，忠實反映表 1 中 `airspeed_sensor_validated=false`、`battery_monitor_validated=false`、
`c2_link_range_km=待填入`、`terrain_clearance_m=待任務定義` 等尚未實測的欄位。另必然出現：

```text
alternate_auto_placed(ALT1, 400 m from home)
fov_width_mismatch(cls=A: 224 m @ 160 m AGL vs UI coverage_width_m=160 m)
fov_width_mismatch(cls=B: 235 m @ 160 m AGL vs UI coverage_width_m=160 m)
```

`fov_width_mismatch` **不是錯誤而是設計**：UI 的 `coverage_width_m` 只是核對值，真正刈幅由 LiDAR 的 `fov_usable_deg`
與相機的 `sensor_width_mm / focal_length_mm` 在該高度推導（§8 第 3 列）。

---

## 4. 資料流總覽

```
AeroPlan UI：corners（main_window.py:148）· home_point（:205）· get_dccpp_params() 25 鍵（parameter_panel.py:2488）
  → pyqtSignal(dict)（parameter_panel.py:56 / :2527）→ connect（main_window.py:486）
  → _on_dccpp_coverage_requested（main_window.py:4673）
       areas  = [{'area_id','polygon','priority'}]        main_window.py:4709
       drones = [{'drone_id','name','position',...}]      main_window.py:4721-4744
  （§7：改由 HvPlanningWorker(QThread) 承接）
       ▼
接縫 SwarmCoordinator.plan_coverage_dccpp（swarm_coordinator.py:975）
  ★ 方式 A：於 :1076 插入 hv 分支（DEM 已在 :1064-1075 轉成 terrain_func）
  ★ 方式 B：另開 signal，完全不碰這個檔案
       ▼
hv_dccpp/integration/aeroplan_adapter.py — HvDccppCoordinator.plan_coverage_dccpp
  ① to_mission_inputs → MissionInputs, id_map={drone_id: uav_id}
       class_of→'A'/'B'；terrain_func_to_dem→DemGrid；fence_zones_to_nofly→NoFlyZone；
       _default_payload_reqs→PayloadRequirement×1–2；_resolve_alternates→Site
  ② context.build_context(inputs, cfg) → MissionContext（地形／風／能量／感測器）
  ③ planner.plan_mission(...) → MissionPlan（每機 4-D 航點）
       decompose → layering → corridors → cache → allocate → idp → assemble_3d → deconflict
  ④ from_mission_plan → 6 鍵 dict（HvBuiltPath / HvBuiltWaypoint / HvCoveragePath /
       HvOperationSegment / HvUavState / HvMdtspResult；ardupilot.reduce_waypoints 縮點 494→74）
       ▼  {'results','allocation','coverage_paths','uav_states','total_makespan','assembled_paths'}
AeroPlan 下游（★ 全部不用改）
  NFZ 重繞 :4844 · 防撞 :4852-4870 · 存結果 :5014 · 地圖 :5015 · 統計 :5046-5054 · .waypoints :5213
    航向 getattr(wp,'heading_compass_deg',getattr(wp,'heading_deg',0.0))  :5308
    長度 getattr(apath,'takeoff_length_m',0.0) …                         :5368-5372
  VTOL 匯出 vtol_mission_exporter.py:219 · SITL 上傳 :6041-6210 → sitl_link.py:264 → _send_mission :1214
  SITL spawn :6330-6356（由 assembled_paths 推各機 home）

回程：SITLLink → FleetRegistry（fleet_registry.py:24）.snapshot()（:106）
  → telemetry_frames_to_events → [LinkLoss|LinkRestored|EnergyUpdate], {uav_id: Telemetry}
  → dynamic.replan(...) → ReplanResult → from_mission_plan → 新 6 鍵 dict → 只重傳 changed_uavs
```

---

## 5. 接入方式 A（推薦，最小改動）

### 5.1 原則與插入點

在 `REPO/mission/swarm_coordinator.py:975 plan_coverage_dccpp` **內部**加分支：條件成立走 hv，否則原封不動走既有 v2 路徑。
回傳結構相同 → **下游一行不用改**。插入點選 `:1076`（DEM 載入區塊 `:1064-1075` 之後、建立 `UAVState` 的 `:1077` 之前）：
此處 `dem_path` 已被 `:1070 terrain_func = dem_mgr.get_elevation` 轉成回呼，hv 可直接吃（§11.1）。

### 5.2 可直接貼的 diff

```diff
--- a/mission/swarm_coordinator.py
+++ b/mission/swarm_coordinator.py
@@ -1075,6 +1075,58 @@ class SwarmCoordinator:
                 _log.getLogger(__name__).warning(f"DEM 載入失敗: {e}")
 
+        # ══ HV-DCCPP 分支（異構雙構型 VTOL）；見 docs/AEROPLAN_接入手冊.md §5 ══
+        #    回傳與下方 v2 路徑完全相同的 6 鍵字典，下游不需修改。
+        if getattr(self, 'use_hv_dccpp', False):
+            try:
+                # ★ AeroPlan 有兩個 PlannerConfig（planner_base.py:70、heterogeneous_nfz_planner.py:80）
+                #   → 一律用別名匯入（§17.2）
+                from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator, to_planner_config
+            except ImportError as _e:
+                import logging as _log
+                _log.getLogger(__name__).warning(f"hv_dccpp 未安裝，降級為既有 DCCPP：{_e}")
+            else:
+                try:
+                    from mission.fence_zone import FenceZoneRegistry, ZoneCategory
+                    _r = FenceZoneRegistry.instance()          # fence_zone.py:181
+                    _zones = _r.by_category(ZoneCategory.NFZ) + _r.by_category(ZoneCategory.THREAT)
+                except Exception:
+                    _zones = []
+                _p = getattr(self, 'hv_params', None) or {}
+                _hv = HvDccppCoordinator(
+                    class_of=getattr(self, 'hv_class_of', None),      # ★ 必填，見 §10.3
+                    cfg=to_planner_config(_p), fence_zones=_zones,
+                    home=getattr(self, 'hv_home', None),
+                    alternates=getattr(self, 'hv_alternates', None),
+                    wind=getattr(self, 'hv_wind', None),              # (speed_mps, dir_from_deg)
+                    c2_range_km=float(_p.get('c2_range_km', 25.0)),
+                    terrain_clearance_m=_p.get('terrain_clearance_m'),
+                    waiver_max_agl_m=float(_p.get('waiver_max_agl_m', 200.0)),
+                    dem_step_m=float(_p.get('dem_step_m', 50.0)),
+                    dem_margin_m=float(_p.get('dem_margin_m', 3000.0)),   # ★ 不要用預設 1500（§11.1）
+                    progress=getattr(self, 'hv_progress', None),
+                    on_report=getattr(self, 'hv_on_report', None), raise_on_no_go=True)
+                self.hv_coordinator = _hv       # 供匯出／上傳／重規劃取用（§12、§13）
+                result = _hv.plan_coverage_dccpp(
+                    areas=areas, drones=drones, coverage_width_m=coverage_width_m,
+                    overlap_rate=overlap_rate, auto_scan_angle=auto_scan_angle,
+                    coverage_altitude=coverage_altitude,
+                    terrain_func=terrain_func,      # ★ DEM 已在 :1064-1075 轉成回呼
+                    enable_altitude=enable_altitude, mounting_angle_deg=mounting_angle_deg,
+                    dem_path="",                    # 已轉成 terrain_func，不重複讀
+                    coordination_mode=coordination_mode,
+                    runway_bearing_deg=runway_bearing_deg, runway_length_m=runway_length_m,
+                    landing_bearing_deg=landing_bearing_deg, landing_rollout_m=landing_rollout_m)
+                import logging as _log
+                _log.getLogger(__name__).info(
+                    f"HV-DCCPP: makespan={result['total_makespan']:.1f}s "
+                    f"({len(result.get('assembled_paths') or {})} 機) notes={len(_hv.notes)}")
+                return result
+
         # ── 建立 UAVState 物件 ────────────────────────────
         uav_states: List[UAVState] = []
```

規劃失敗（如 preflight `NO_GO`）時 `AeroPlanAdapterError` 往上拋，被 `REPO/ui/main_window.py:5057` 的
`except Exception` 接住跳 `QMessageBox`——這是**正確**行為：NO_GO 代表任務不可行（缺備降點、能量不足…），
不該偷偷退回一個「看起來能飛」的舊結果。要靜默降級就再包一層 `except AeroPlanAdapterError: pass`，
但**務必把錯誤顯示給操作員**。

### 5.3 旗標

`REPO/mission/swarm_coordinator.py:395 __init__` 加：`use_hv_dccpp=False`（由呼叫端開啟）、
`hv_class_of=None`、`hv_params={}`、`hv_home=None`、`hv_alternates=None`、`hv_wind=None`、
`hv_progress=None`、`hv_on_report=None`、`hv_coordinator=None`。
在 `REPO/ui/main_window.py:4754` 建構處開啟——**注意 `vehicle_type` 閘門**（hv 只支援 VTOL）：

```python
            coordinator = SwarmCoordinator()
            coordinator.use_hv_dccpp = (
                vehicle_type == 'vtol' and bool(getattr(self, 'hv_dccpp_enabled', False)))
            if coordinator.use_hv_dccpp:
                coordinator.hv_params = dict(params)
                coordinator.hv_class_of = self._hv_class_of      # §10.3
                coordinator.hv_home = self.home_point            # main_window.py:205
                coordinator.hv_progress = lambda f, m: self.statusBar().showMessage(
                    f"HV-DCCPP {f * 100:.0f}%：{m}", 0)
                coordinator.hv_on_report = self._on_hv_report    # §6.4
```

`hv_dccpp_enabled` 建議做成選單勾選項，或 `REPO/config/settings.py:241 GlobalSettings` 的新區段
（⚠ 該檔**新增區段要改 4 處**：dataclass、`__init__ :252-259`、`load() :282-301`、`save() :317-325`）。
最省事：`self.hv_dccpp_enabled = os.environ.get('AEROPLAN_HV_DCCPP', '') == '1'`。

### 5.4 為什麼下游全部不用改（逐一驗證）

| 下游 | 位置 | 讀什麼 | 轉接層提供什麼 |
|---|---|---|---|
| NFZ 重繞 | `main_window.py:4844` | `assembled_paths` 的 `waypoints[].lat/lon` | 有。hv 已避開 NFZ → 這層通常不動航點；若動了會微幅拉長且 `*_length_m` 不更新，A/B 比較時請關掉 |
| 防撞後處理 | `main_window.py:4852-4870` | 同上（`collision_avoidance` 預設 False） | hv 已 4-D 分離；**建議 hv 分支下取消勾選**，否則會再疊一次高度偏移 |
| 存結果 / 地圖 / 統計 | `:5014` / `:5015`（`swarm_data` 於 `:4872-5013` 組裝）/ `:5046-5054` | 整個 dict；`lat/lon/alt`、`segment_type.name`、`total_length_m`、`total_makespan` | 6 鍵齊全（`test_returns_six_keys`）；`segment_type.name` 只會是那 5 個字串（`test_segment_labels_only_five_strings`）；`total_makespan` 是 float 但語意不同（§9.5） |
| `.waypoints` | `:5213`；`:5308` 讀航向、`:5368-5372` 讀五個長度 | `getattr` 雙讀 | 航點**同時**有 `heading_compass_deg` 與 `heading_deg`（property 別名）；**五個** `*_length_m` 都是真欄位（`test_duck_typing_getattr_pattern`） |
| VTOL 匯出 | `vtol_mission_exporter.py:219` | 同上 | `alt` 為 AGL 相對 home，符合該檔 `:32 _FRAME_GLOBAL_REL_ALT = 3` |
| SITL 上傳 | `:6041-6043` → `:6045-6159` → `:6188-6210` 依 `link.sysid_label` 派送 | `assembled_paths` 鍵須為 `int` | 鍵是 AeroPlan 的 `drone_id`（int）。**但 `_build_wps_for_path` 有起飛 bug，見 §12.3** |
| SITL spawn | `:6330-6356` | `waypoints[0]` / `[1]` 的 lat/lon | 有；首航點為起飛點，`alt ≈ 0`（`test_waypoint_alt_is_relative_to_home`，`abs=2.0`） |

### 5.5 既有測試

`use_hv_dccpp` 預設 `False` → `REPO/tests/test_dccpp_pipeline.py:16-23` 的 golden values **不受影響**。
且 hv 用橢球、AeroPlan 部分模組用 111111.0 m/deg（差 0.1%），**那些 golden 本來就不能驗收 hv 分支**（§17.4）。
請另建 `REPO/tests/test_hv_dccpp_adapter.py`。

---

## 6. 接入方式 B（並存，A/B 比較）

新增 UI 分頁 + 新 signal + 新 handler，**完全不碰 `swarm_coordinator.py`**
（`REPO/ui/main_window.py:5046-5054` 已有 `_dccpp_metrics` 就是為了比較）。

**① 新 signal**（`REPO/ui/widgets/parameter_panel.py:56` 旁）：

```python
    dccpp_coverage_requested = pyqtSignal(dict)    # 既有
    hv_dccpp_requested = pyqtSignal(dict)          # ← 新增
```

**② 新面板**（`REPO/ui/widgets/parameter_panel.py:1956 create_dccpp_panel` 旁；該檔已 4461 行、是待拆的 God Object
→ **建議獨立分頁**）。骨架：

```python
    def create_hv_dccpp_panel(self) -> QGroupBox:
        """HV-DCCPP 面板（A = Pioneer + LiDAR、B = HERO + 相機）。寫法比照 create_dccpp_panel（:1956）。"""
        from PyQt6.QtWidgets import QCheckBox, QDoubleSpinBox, QFormLayout, QGroupBox, QPushButton, QSpinBox
        group = QGroupBox("=== HV-DCCPP 異構雙構型 VTOL ===")
        group.setObjectName("hvDccppGroup")
        form = QFormLayout(group)

        def _spin(lo, hi, val, step=1.0, suffix="", tip=""):
            w = QDoubleSpinBox(); w.setRange(lo, hi); w.setSingleStep(step); w.setValue(val)
            if suffix: w.setSuffix(suffix)
            if tip: w.setToolTip(tip)
            return w

        self.hv_num_a_spin = QSpinBox(); self.hv_num_a_spin.setRange(0, 8); self.hv_num_a_spin.setValue(1)
        self.hv_num_b_spin = QSpinBox(); self.hv_num_b_spin.setRange(0, 8); self.hv_num_b_spin.setValue(1)
        self.hv_altitude_spin = _spin(50, 400, 160, 10, " m AGL",
            "A 的 min_fw_operation_agl_m = 120 m（平台參數表 表 1），低於此值會被自動夾住")
        self.hv_overlap_spin = _spin(0.0, 0.9, 0.25, 0.05)
        self.hv_waiver_spin  = _spin(120, 500, 200, 10, " m AGL", "無豁免時填 120；A 在平地將無高度帶可用")
        self.hv_c2_spin      = _spin(1, 200, 25, 1, " km")       # 表 1 c2_link_range_km = 待填入
        self.hv_clear_spin   = _spin(0, 200, 0, 1, " m"); self.hv_clear_spin.setSpecialValueText("自動")
        self.hv_wind_v_spin  = _spin(0, 25, 0, 0.5, " m/s")
        self.hv_wind_d_spin  = _spin(0, 359, 0, 5, " °（風從何方吹來）")
        self.hv_min_sep_spin = _spin(20, 1000, 100, 10, " m")    # → PlannerConfig.d_sep_m
        self.hv_alt_off_spin = _spin(10, 200, 50, 5, " m")       # → PlannerConfig.dh_sep_m
        self.hv_reduce_check = QCheckBox("縮點（建議開）"); self.hv_reduce_check.setChecked(True)
        for label, w in (("構型 A（Pioneer + LiDAR）台數", self.hv_num_a_spin),
                         ("構型 B（HERO + 相機）台數", self.hv_num_b_spin),
                         ("作業高度", self.hv_altitude_spin), ("側向重疊率", self.hv_overlap_spin),
                         ("豁免後 AGL 上限", self.hv_waiver_spin), ("C2 鏈路半徑", self.hv_c2_spin),
                         ("對地淨空", self.hv_clear_spin), ("平均風速", self.hv_wind_v_spin),
                         ("風向", self.hv_wind_d_spin), ("水平分離下限 d_sep", self.hv_min_sep_spin),
                         ("垂直分層間距 dh_sep", self.hv_alt_off_spin), ("", self.hv_reduce_check)):
            form.addRow(label, w)
        btn = QPushButton("HV-DCCPP 規劃")
        btn.clicked.connect(lambda: self.hv_dccpp_requested.emit(self.get_hv_dccpp_params()))
        form.addRow(btn)
        return group

    def get_hv_dccpp_params(self) -> dict:
        """鍵名刻意與 get_dccpp_params()（:2488）不重疊。"""
        clr = self.hv_clear_spin.value()
        return {'num_a': self.hv_num_a_spin.value(), 'num_b': self.hv_num_b_spin.value(),
                'coverage_altitude': self.hv_altitude_spin.value(),
                'overlap_rate': self.hv_overlap_spin.value(),
                'waiver_max_agl_m': self.hv_waiver_spin.value(),
                'c2_range_km': self.hv_c2_spin.value(),
                'terrain_clearance_m': (clr if clr > 0.0 else None),
                'wind': (self.hv_wind_v_spin.value(), self.hv_wind_d_spin.value()),
                'min_separation_m': self.hv_min_sep_spin.value(),   # → d_sep_m
                'alt_offset_m': self.hv_alt_off_spin.value(),       # → dh_sep_m
                'reduce_waypoints': self.hv_reduce_check.isChecked()}
```

**③ connect**（`REPO/ui/main_window.py:486` 旁）：
`self.parameter_panel.hv_dccpp_requested.connect(self._on_hv_dccpp_requested)`

**④ handler**（`REPO/ui/main_window.py:4673` 旁）：

```python
    def _on_hv_dccpp_requested(self, params: dict):
        from PyQt6.QtWidgets import QMessageBox
        from ui.workers.planning_worker import HvPlanningWorker
        if len(self.corners) < 3:
            QMessageBox.warning(self, "邊界不足", "請先在地圖上標出至少 3 個邊界點"); return
        areas = [{'area_id': 1, 'polygon': list(self.corners), 'priority': 1.0}]
        if self.home_point is not None:                                   # main_window.py:205
            base_lat, base_lon = self.home_point
        else:
            base_lat = sum(c[0] for c in self.corners) / len(self.corners)
            base_lon = sum(c[1] for c in self.corners) / len(self.corners)
        drones, classes = [], {}
        n_a, n_b = int(params['num_a']), int(params['num_b'])
        for i in range(n_a + n_b):
            cls, did = ('A' if i < n_a else 'B'), i + 1
            classes[did] = cls
            drones.append({'drone_id': did, 'name': f'{cls}{did}', 'position': (base_lat, base_lon),
                           'speed': 21.5 if cls == 'A' else 19.0, 'heading': 0.0,
                           'turn_radius': 200.0 if cls == 'A' else 120.0,
                           'vehicle_type': 'vtol', 'fov_width': 0.0,
                           'hv_class': cls})          # ← default_class_of 會讀這個鍵
        self._hv_classes, self._hv_areas, self._hv_drones = classes, areas, drones
        self._hv_worker = HvPlanningWorker(           # ★ 必須存成成員，否則被 GC 回收
            areas=areas, drones=drones, params=params,
            class_of=lambda d: classes[int(d['drone_id'])],
            terrain_func=self._terrain_func_or_none(),      # §11.1
            fence_zones=self._nfz_zones(),                  # §11.2
            home=self.home_point, parent=self)
        self._hv_worker.progress.connect(
            lambda f, m: self.statusBar().showMessage(f"HV-DCCPP {f * 100:.0f}%：{m}", 0))
        self._hv_worker.finished_ok.connect(self._on_hv_finished)
        self._hv_worker.failed.connect(self._on_hv_failed)
        self._hv_worker.report.connect(self._on_hv_report)
        self._hv_worker.start()

    def _on_hv_finished(self, result: dict):
        self._hv_coordinator = self._hv_worker.coordinator    # 供 §12.3 上傳、§13 重規劃
        self._dccpp_result = result                           # 同 main_window.py:5014
        ap = result.get('assembled_paths') or {}
        self._hv_metrics = {'algorithm': 'HV-DCCPP (異構雙構型 VTOL)', 'num_drones': len(ap),
                            'total_waypoints': sum(len(p.waypoints) for p in ap.values()),
                            'total_distance': sum(p.total_length_m for p in ap.values()),
                            'makespan': result['total_makespan']}
        self._display_dccpp_result(result)     # ← 把 main_window.py:4844-5015 抽成的方法
        self.statusBar().showMessage(f"HV-DCCPP 完成 | {len(ap)} 台 | "
                                     f"Makespan {result['total_makespan']:.1f}s", 10000)

    def _on_hv_failed(self, msg: str):
        from PyQt6.QtWidgets import QMessageBox
        self.statusBar().clearMessage(); QMessageBox.critical(self, "HV-DCCPP 失敗", msg)

    def _on_hv_report(self, payload: dict):
        """payload = {'status','errors','warnings','notes','makespan_s','timing_ms','uncovered'}。"""
        logger.info(f"[HV-DCCPP] status={payload['status']} errors={len(payload['errors'])} "
                    f"warnings={len(payload['warnings'])} total={payload['timing_ms'].get('total', 0.0):.0f} ms")
        for w in payload['warnings']:
            logger.warning(f"[HV-DCCPP] {w}")
        if payload['uncovered']:
            logger.error(f"[HV-DCCPP] 未覆蓋：{payload['uncovered']}")   # ← 應紅字顯示給操作員
```

**新增一個參數的三步驟**（比照既有 `create_dccpp_panel :1956` → `get_dccpp_params :2488` → `main_window.py:4695-4706`）：
① 在 `create_hv_dccpp_panel()` 建 widget；② 在 `get_hv_dccpp_params()` 加一鍵；
③ 在 `_on_hv_dccpp_requested` 傳給 worker → `HvDccppCoordinator(...)` 的對應參數。
若要調的是 `PlannerConfig` 欄位，**不必改 adapter**——用萬用逃生口
`params['hv_cfg'] = {'seed': 7, 'B': 6, 'dt_dep_s': 90.0}`（餵 `PlannerConfig.from_dict`；未知鍵丟 `ValueError`，刻意）。
---

## 7. 執行緒（必做）

### 7.1 為什麼一定要做

AeroPlan 的規劃**完全同步在 GUI 執行緒**：`REPO/ui/main_window.py:4760-4776` 直接呼叫，前面只有 `:4749-4751`
一次 `QApplication.processEvents()`。全樹只有三個 `QThread`（`REPO/mission/sitl_link.py:163 SITLLink`、
`mock_sitl_link.py:52`、`tools/swarm_perf_profiler.py:97`）。`REPO/core/base/planner_base.py:78-83` 有
`progress_callback`、`:125` 有 `cancel()`，但**無人提供也無人檢查**；`REPO/config/settings.py:129-130` 的
`enable_multiprocessing`/`max_workers` 是死設定。

hv 規劃耗時實測：§3 情境（2 機 7 條帶）**1.20 s**；`examples/run_hills.py`（**8 機 107 條帶**）腳本含繪圖 **42.8 s**。
1.2 s 可忍受，**43 s 會讓 Windows 標題列顯示「沒有回應」**。QThread 不是可選項。

### 7.2 `REPO/ui/workers/planning_worker.py`（完整可貼）

新建 `REPO/ui/workers/__init__.py`（空檔）與：

```python
"""ui/workers/planning_worker.py — HV-DCCPP 規劃背景執行緒。

寫法比照 REPO/mission/sitl_link.py:163 SITLLink：繼承 QThread、對外只走 pyqtSignal、
建構子只存參數、例外一律轉成 failed signal。
理由見 docs/AEROPLAN_接入手冊.md §7（main_window.py:4753-4776 同步阻塞，大情境數十秒）。
"""
from __future__ import annotations

from typing import Any, Callable, Optional, Sequence

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger      # ui/ 與 mission/ 的 logging 慣例（§17.6）

logger = get_logger(__name__)


class HvPlanningWorker(QThread):
    """背景執行 hv_dccpp 覆蓋規劃。

    信號：progress(fraction 0–1, 訊息)／finished_ok(6 鍵 dict)／failed(錯誤字串)／
          report({'status','errors','warnings','notes','makespan_s','timing_ms','uncovered'})
    """

    progress = pyqtSignal(float, str)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)
    report = pyqtSignal(dict)

    def __init__(self, areas: list[dict], drones: list[dict], params: dict, *,
                 class_of: Optional[Callable[[dict], str]] = None,
                 terrain_func: Optional[Callable[[float, float], float]] = None,
                 fence_zones: Optional[Sequence[Any]] = None,
                 home: Optional[tuple[float, float]] = None,
                 alternates: Optional[Sequence[Any]] = None, parent=None) -> None:
        super().__init__(parent)
        self._areas, self._drones, self._params = list(areas), list(drones), dict(params)
        self._class_of, self._terrain_func = class_of, terrain_func
        self._fence_zones = list(fence_zones) if fence_zones else None
        self._home = home
        self._alternates = list(alternates) if alternates else None
        self._cancelled = False
        self.coordinator = None          # 規劃後填入，供匯出／上傳／重規劃取用
        self.result: Optional[dict] = None

    def cancel(self) -> None:
        """要求取消；只會在下一次 progress 回呼時拋出而中止，不保證即時（§7.5）。"""
        self._cancelled = True

    def _tick(self, frac: float, msg: str) -> None:
        if self._cancelled:
            raise RuntimeError("使用者取消規劃")
        self.progress.emit(float(frac), str(msg))

    def run(self) -> None:
        try:
            # ★ 名稱衝突：AeroPlan 有兩個 PlannerConfig → 一律用別名匯入（§17.2）
            from hv_dccpp.integration.aeroplan_adapter import (
                AeroPlanAdapterError, HvDccppCoordinator, to_planner_config)
        except ImportError as e:
            self.failed.emit(f"hv_dccpp 未安裝或匯入失敗：{e}")
            return
        try:
            p = self._params
            coord = HvDccppCoordinator(
                class_of=self._class_of,
                cfg=to_planner_config(p),                     # min_separation_m / alt_offset_m / hv_cfg
                fence_zones=self._fence_zones, home=self._home, alternates=self._alternates,
                wind=p.get('wind'),                           # (speed_mps, dir_from_deg) 或 None
                c2_range_km=float(p.get('c2_range_km', 25.0)),
                terrain_clearance_m=p.get('terrain_clearance_m'),
                waiver_max_agl_m=p.get('waiver_max_agl_m', 200.0),
                dem_step_m=float(p.get('dem_step_m', 50.0)),
                dem_margin_m=float(p.get('dem_margin_m', 3000.0)),   # ★ 不要用預設 1500（§11.1）
                reduce_waypoints=bool(p.get('reduce_waypoints', True)),
                progress=self._tick, on_report=self.report.emit, raise_on_no_go=True)
            self.coordinator = coord
            result = coord.plan_coverage_dccpp(
                areas=self._areas, drones=self._drones,
                coverage_width_m=float(p.get('fov_width_m', 0.0) or 0.0),
                overlap_rate=float(p.get('overlap_rate', 0.1)),
                auto_scan_angle=bool(p.get('auto_scan_angle', True)),
                coverage_altitude=float(p.get('coverage_altitude', p.get('altitude', 100.0))),
                terrain_func=self._terrain_func,
                enable_altitude=bool(p.get('enable_altitude', True)),
                mounting_angle_deg=float(p.get('mounting_angle_deg', 0.0)),
                dem_path="",                                  # 一律用 terrain_func（§11.1）
                coordination_mode=str(p.get('coordination_mode', 'uncoordinated')))
            self.result = result
            logger.info(f"[HV-DCCPP] 完成：makespan={result['total_makespan']:.1f}s "
                        f"{len(result.get('assembled_paths') or {})} 機")
            self.finished_ok.emit(result)
        except AeroPlanAdapterError as e:
            detail = "\n".join(f"  • {x}" for x in getattr(e, 'errors', []) or [])
            logger.error(f"[HV-DCCPP] 規劃失敗：{e}")
            self.failed.emit(f"{e}\n{detail}" if detail else str(e))
        except Exception as e:                                # noqa: BLE001 - 執行緒不得帶例外死掉
            logger.error(f"[HV-DCCPP] 未預期錯誤：{e}", exc_info=True)
            self.failed.emit(f"HV-DCCPP 未預期錯誤：{e}")
```

### 7.3 `REPO/ui/main_window.py:4753-4776` 怎麼改

把 `:4844` 之後的**全部後處理**抽成 `_display_dccpp_result(self, dccpp_result)`，規劃交給 worker：

```python
            self.statusBar().showMessage("HV-DCCPP 規劃中…", 0)
            from ui.workers.planning_worker import HvPlanningWorker
            self._hv_worker = HvPlanningWorker(
                areas=areas, drones=drones, params=params,
                class_of=self._hv_class_of,                  # §10.3
                terrain_func=self._terrain_func_or_none(),   # §11.1
                fence_zones=self._nfz_zones(),               # §11.2
                home=self.home_point, parent=self)
            self._hv_worker.progress.connect(
                lambda f, m: self.statusBar().showMessage(f"HV-DCCPP {f * 100:.0f}%：{m}", 0))
            self._hv_worker.finished_ok.connect(self._display_dccpp_result)
            self._hv_worker.failed.connect(self._on_hv_failed)
            self._hv_worker.report.connect(self._on_hv_report)
            self._hv_worker.start()
            return          # ★ 後處理改由 finished_ok 觸發
```

三件必須注意：

1. **`self._hv_worker` 必須存成成員**——區域變數會被 GC 回收，`QThread` 直接消失。
2. **`parent=self`** 讓 Qt 管生命週期；視窗關閉時 `if self._hv_worker: self._hv_worker.wait(3000)`。
3. **worker 內不可碰任何 widget**——只做純運算並 emit signal（同 `REPO/docs/PROJECT_STATUS.md:29-45`「core/ 不得依賴 Qt」的原則）。

### 7.4 進度回報從哪來

`HvDccppCoordinator._tick` 在六個點呼叫 `progress`：

| fraction | 訊息 | 階段 |
|---|---|---|
| 0.02 | 組裝任務輸入 | `to_mission_inputs`（**DEM 取樣在這裡**，大範圍會慢，§11.1） |
| 0.15 | 建立情境（地形／風場／能量模型） | `build_context` |
| 0.35 | 前置檢查（preflight） | — |
| 0.40 | 規劃中（分解 → 配置 → IDP → 組裝 → 去衝突） | `plan_mission`（**最長，中間無進度**） |
| 0.90 | 轉換為 AeroPlan 結果格式 | `from_mission_plan`（含縮點） |
| 1.00 | 完成 | — |

`test_progress_and_report_callbacks` 釘住「單調遞增且最後為 1.0」。
⚠ 0.40 → 0.90 之間沒有中間進度 → UI 請用不定量 `QProgressBar`（`setRange(0, 0)`），不要用百分比條。
規劃完成後 `on_report` 的 `timing_ms` 會給逐段耗時，可事後顯示。

### 7.5 取消的做法與限制

`plan_mission` **沒有取消點**。上面的 `cancel()` 是「下一次 `progress` 回呼時拋例外」，所以：

* 可取消時機只有那 6 個 `_tick` 點；**0.40 → 0.90 的 `plan_mission` 全段不可取消**（山丘情境按下取消仍需等數十秒）。
* **絕對不要** `QThread.terminate()`：會在 numpy/scipy 的 C 層留下不一致狀態。
* 別在取消後 `wait()`，那會凍結 UI——等 `failed` signal 進來即可。

要「按下就停」，唯一乾淨的做法是丟到**子行程**（`multiprocessing`）用 `terminate()` 殺掉。
`MissionInputs` 與 6 鍵字典都是純資料可 pickle；但 `terrain_func`（bound method）**不可 pickle**，
必須先在主行程用 `terrain_func_to_dem` 取樣成 `DemGrid`（純 numpy）再傳過去。

---

## 8. 參數對照表

`REPO/ui/widgets/parameter_panel.py:2488 get_dccpp_params()` 實際回傳 **25 鍵**（`:2491-2521`；口語常說「22 鍵」）。
`aeroplan_adapter.PARAM_KEY_NOTES` 對這 25 鍵逐一交代，`test_to_planner_config_maps_two_keys` 斷言
「每個 UI 鍵都必須在 `PARAM_KEY_NOTES` 中有交代」。

| # | UI 鍵 | 單位 | UI 預設 | 去向與處置 |
|---|---|---|---|---|
| 1 | `vehicle_type` | — | `multirotor` | **忽略**（hv 只支援 4+1 VTOL；構型由 `class_of` 決定）。§5.3 拿它當 hv 分支的閘門 |
| 2 | `num_drones` | 台 | 3 | **忽略**（機隊由 `drones` 決定） |
| 3 | `fov_width_m` | m | 100 | **核對值**。實際刈幅由感測器 + 高度推導（規格 §4.2/§4.3）；差 > 25% 記 `fov_width_mismatch` |
| 4 | `overlap_rate` | 比例 | 0.1 | ✅ → `PayloadRequirement.side_overlap`。須在 `[0,1)`，否則採 §16.2 預設並記 `overlap_rate_out_of_range` |
| 5 | `altitude` | m AGL | 100 | ✅ → `PayloadRequirement.h_target_agl_m`，**但會被夾**到 `[max_c min_fw_operation_agl_m, waiver_max_agl_m]`。⚠ **UI 預設 100 m 一定會被夾到 120 m**（A 的下限），記 `coverage_altitude_clamped(100.0->120.0; …)` |
| 6 | `speed` | m/s | 15 | **忽略**（平台常數 `spec.cruise_airspeed_mps`：A 21.5、B 19.0） |
| 7 | `turn_radius` | m | 50 | **忽略**（逐構型 `spec.turn_radius_planning_m`：A 200、B 120，再經風修正得 `r_eff`） |
| 8 | `auto_scan_angle` | bool | True | **忽略**（hv 一律最佳化掃描方向 α*，規格 §7.3）。`False` 記 `auto_scan_angle_ignored` |
| 9 | `enable_altitude` | bool | True | **忽略**（高度規劃不可關，規格 §9、Alg.3）。`False` 記 `enable_altitude_ignored` |
| 10 | `auto_landing` | bool | True | **忽略**（一律規劃 RTB + 降落走廊，規格 §11） |
| 11 | `mounting_angle_deg` | ° | 0 | **條件生效**：須同時 `hfov_deg > 0` 且 `vfov_deg > 0`（且 `cfg.fov_model="trapezoid"`）才建 `TrapezoidFovSpec`；單獨給只記 note |
| 12 | `hfov_deg` | ° | — | 同上 → `TrapezoidFovSpec.h_fov_deg`（論文 Eq.(2)(3)） |
| 13 | `vfov_deg` | ° | — | 同上 → `TrapezoidFovSpec.v_fov_deg` |
| 14 | `coordination_mode` | 字串 | `uncoordinated` | **忽略**（hv 一律 4-D 去衝突，規格 §10.6、D14）。非預設值記 `coordination_mode_ignored` |
| 15 | `dem_path` | 路徑 | `''` | **忽略**（本層不讀 raster，避免 rasterio 相依）。非空記 `dem_path_ignored`；改傳 `terrain_func`（§11.1） |
| 16 | `takeoff_bearing` | ° | 0 | **幾乎忽略**：只在 drone dict 缺 `heading` 時當初始航向 |
| 17 | `runway_length` | m | 500 | **忽略**（VTOL 不需跑道），記 `runway_length_m_ignored` |
| 18 | `landing_bearing` | ° | 180 | **忽略**（進場方位由 D19 逆風規則決定），記 `landing_bearing_deg_ignored` |
| 19 | `pattern_alt` | m | 80 | **忽略** |
| 20 | `landing_rollout` | m | 0 | **忽略**（垂直降落無滑行），記 `landing_rollout_m_ignored` |
| 21 | `takeoff_spacing` | m | 50 | **忽略**（只影響 UI 佈機）。起飛**時距**是 `PlannerConfig.dt_dep_s`（預設 60.0 s，D46） |
| 22 | `collision_avoidance` | bool | False | **忽略**（分離一律啟用）。⚠ hv 分支下建議在 UI 取消勾選，避免 `main_window.py:4852-4870` 對已分離的路徑再疊高度偏移 |
| 23 | `min_separation_m` | m | 100 | ✅ **真的生效**（> 0 時）→ `PlannerConfig.d_sep_m`（hv 預設 100.0） |
| 24 | `alt_offset_m` | m | 30 | ✅ **真的生效**（> 0 時）→ `PlannerConfig.dh_sep_m`（hv 預設 50.0） |
| 25 | `avoid_strategy` | 字串 | `altitude` | **忽略** |

**結論：25 鍵中只有 4 個真正影響結果**（`overlap_rate`、`altitude`、`min_separation_m`、`alt_offset_m`），
另 3 個條件生效。其餘 18 鍵不是被平台常數取代，就是被「hv 一律做」的設計取代。

### 8.1 萬用逃生口 `hv_cfg`

```python
params['hv_cfg'] = {'d_sep_m': 150.0, 'dt_dep_s': 90.0, 'B': 6, 'n_local_search': 400,
                    'seed': 7, 'dem_step_m': 25.0}
cfg = to_planner_config(params)     # 直接餵 PlannerConfig.from_dict；未知鍵丟 ValueError（刻意）
```

⚠ `HvDccppCoordinator` 的 `cfg` 預設是 `HvPlannerConfig(dem_step_m=25.0)`，不是 `PlannerConfig()` 的 **10.0**。
若你自己傳 `cfg=to_planner_config(params)`，拿到的是 10.0，`build_context` 的地形網格節點數變 6.25 倍。要保持 25.0：

```python
cfg = to_planner_config(params, base=HvPlannerConfig(dem_step_m=25.0))
```

### 8.2 UI 應新增哪些欄位

| 欄位 | 為什麼需要 | 對應 API |
|---|---|---|
| 構型 A / B 台數（或逐機構型表） | 只有一個 `num_drones`，無法表達異構機隊 | `drones` + `class_of` |
| 逐構型作業高度 | A 下限 120 m、B 下限 100 m | `payload_reqs={'req_lidar': …, 'req_rgb': …}` |
| 豁免後 AGL 上限 | 沒有豁免（120 m）時 A 在平地**沒有高度帶可用** | `waiver_max_agl_m` |
| C2 鏈路半徑 | 表 2 必填；缺則 preflight `NO_GO`（表 1 為「待填入」） | `c2_range_km` |
| 對地淨空 | D11 必填（表 1 為「待任務定義」） | `terrain_clearance_m` |
| 風速／風向 | 影響 `r_eff`、地速、耗時、逆風進場 | `wind=(speed_mps, dir_from_deg)` |
| 逐構型 AUW / 可用 Wh / 保留比 | 表 2 明列；`DEFAULT_ENERGY_A/B` 只是基準值 | `energy={'A': {...}, 'B': {...}}` |
| 備降點（地圖點選、可複數） | 表 1 `alternate_landing_sites_required=true`；未給會自動放一點（多半不是你要的） | `alternates=[(lat, lon), ...]` |
| 感測器規格（LiDAR FOV／相機焦距·像素間距） | 決定刈幅與 GSD | `sensor_a=(LidarHardware(...),)` / `sensor_b=(CameraHardware(...),)` |
| 縮點開關 | 除錯時想看密集取樣 | `reduce_waypoints` |

---

## 9. 型別與慣例對照表

| 項目 | AeroPlan Studio | hv_dccpp | 轉換？ |
|---|---|---|---|
| 地理座標順序 | `(lat, lon)` WGS84 度（`REPO/mission/geofence_manager.py:60` 明文） | 同 | ❌ 不用 |
| 本地座標 | ENU 公尺 `x=East y=North z=Up`（`REPO/core/geometry/coordinate.py:28-38`） | 同（`geo.GeoFrame.to_enu_xy` / `to_wgs84`） | ❌ 不用 |
| 航向 | 公開資料一律 **compass**（0=北、順時針）；Dubins 內部用 math（0=東，`REPO/core/trajectory/dubins_trajectory.py:52-75`），`_compass_to_math(c)=(90-c)%360`（`dccpp_path_builder.py:86-91`） | `Waypoint.track_deg` 為 compass 度 | ❌ 不用；轉接層以 `_compass()` 正規化到 `[0,360)` |
| 航點航向欄位名 | live `heading_compass_deg`（`dccpp_path_builder.py:54-62`）／legacy `heading_deg`（`dccpp_path_assembler.py:47-63`）；消費端 `main_window.py:5308` 兩個都讀 | — | ✅ **兩個都給**：欄位 + property 別名（恆等，`test_heading_fields_consistent`） |
| 高度（輸出） | 匯出／上傳一律 **AGL 相對 home**（frame 3；`utils/file_io.py:226`、`vtol_mission_exporter.py:32`、`sitl_link.py:1274`） | `alt_msl_m`（MSL）與 `agl_m`（對**當地地形** AGL） | ✅ `alt = max(0, alt_msl_m − home_msl)` |
| 高度（內部） | 🟠 `AltitudePlanner` 內部是 AMSL（`altitude_planner.py:109,177-178`），與匯出的 relative 未協調 | 內部 ENU z（含曲率下垂 A9）；AGL 一律以**自然地形**（`TerrainGrid.raw_view()`）量 | — |
| 相位標籤 | `SegmentLabel` 5 值；消費端比對 `.segment_type.name` **字串**（`main_window.py:6060-6066`、`:6113-6136`） | 14 種 `Phase` | ✅ `PHASE_TO_SEGMENT`（§9.1） |
| 路徑長度欄位 | live `BuiltPath` 只有 entry/operation/transfer + `total_length_m`；legacy `AssembledPath` 另有 takeoff/landing；消費端 `getattr` 讀五個（`main_window.py:5368-5372`） | `UAVPlan.distance_m`（含垂直分量） | ✅ **五個都給**，皆為 ENU **水平**長度 |
| 路徑座標方法 | live `to_latlon()`；legacy `to_latlon_list()` | — | ✅ **三個都給**（另加 `to_latlon_3d_list()`） |
| 載具型別 | `VehicleType{MULTIROTOR, FIXED_WING}`（`mdtsp_solver.py:37-40`，**無 VTOL**）；`'vtol'` 被靜默降級為 `MULTIROTOR`（`swarm_coordinator.py:1080-1082`） | — | ✅ 填 `FIXED_WING`（`HvVehicleType` 另有 `VTOL` 成員供新程式用） |
| 地球半徑 | 🟠 混用 111111.0 m/deg 與橢球 | 全程 WGS84 橢球 | ⚠ 差約 **0.1%**（§9.6） |

### 9.1 相位映射（`PHASE_TO_SEGMENT`，**不可逆**）

| `Phase` | `HvSegmentLabel` | 理由 |
|---|---|---|
| `TAKEOFF`, `TRANS_OUT` | `TAKEOFF` | 垂直起飛與 MC→FW 轉換皆屬起飛序列 |
| `ENTRY`, `CLIMB`, `ORBIT`, `ACCEL` | `ENTRY` | 進場／爬升／盤旋等高／加速 |
| `OPERATE` | `OPERATION` | 條帶掃描（感測器 ON） |
| `TRANSFER`, `HOLD` | `TRANSFER` | 條帶間轉移與等待 |
| `RTB`, `DESCENT`, `DECEL`, `TRANS_IN`, `LAND` | `LANDING` | 返場、下降、減速、FW→MC 轉換、垂直降落 |

AeroPlan 端無法分辨 `ORBIT`（盤旋）與 `CLIMB`（爬升）。要真正的相位請讀
`coordinator.last_plan.plans[uid].waypoints[i].phase`。`test_phase_map_covers_all_14_phases` 釘住 14→5 全覆蓋。

### 9.2 `HvBuiltWaypoint.alt` 到底是什麼

`alt = max(0, alt_msl_m − home_msl)`——**不是**對當地地形的 AGL（起伏地形上 `alt ≠ agl_m`），**不是** AMSL；
就是 `MAV_FRAME_GLOBAL_RELATIVE_ALT`（frame 3）的定義，也是 ArduPilot 接收任務時的期待。
要真 AGL 讀 `coordinator.last_plan.plans[uid].waypoints[i].agl_m`。
`test_waypoint_alt_is_relative_to_home` 釘住首航點 `alt ≈ 0`（`abs=2.0`）與最高點對得上（`abs=5.0`）。

### 9.3 五個 `*_length_m`

第 i 段（`wp[i] → wp[i+1]`）的 ENU **水平**距離 `hypot(Δx, Δy)`，計入 `PHASE_TO_SEGMENT[wp[i].phase]`
——依**出發點**的相位（D45 慣例）。`total_length_m` = 五者之和（`test_five_lengths_and_total`，`rel=1e-12`）。
不含垂直分量 → **必然略小於** `UAVPlan.distance_m`。
**縮點不影響長度統計**：一律以縮點前的全部取樣點計算（`test_reduce_waypoints_shrinks_output`，`rel=1e-9`）。

### 9.4 `HvMdtspResult.makespan` 的單位陷阱

AeroPlan 的**欄位文件**寫「秒」（`REPO/core/global_planner/mdtsp_solver.py:91`），
但**實作**填的是公尺（`REPO/core/dccpp/idp_solver.py:407` 的 `idp_result.makespan_length`）。
`HvMdtspResult.makespan` 填**秒**（與欄位文件一致）。要比長度請讀 `results[aid].total_distances`（`{drone_id: 公尺}`）。

### 9.5 `total_makespan` 的定義差異

AeroPlan：`max(路徑總長 ÷ 速度)`，**不含**起降、MC↔FW 轉換、盤旋等高、錯開起飛（`swarm_coordinator.py:1285-1289`）。
hv：`MissionPlan.makespan_s`，含全部相位 + D46 錯開起飛（規格 §10）。
§3 情境 hv 給 1479.4 s，而 24.5 km ÷ 21.5 m/s ≈ 1142 s——**hv 必然較大，這不是迴歸**。
A/B 比較請比 `total_distances` 或各機 `total_length_m`。

### 9.6 座標 0.1% 差異

`REPO/docs/refactor_audit_2026.md:18-35` 自承 AeroPlan 投影不一致：`region_divider.py:260-283` 用固定
111111.0 m/deg，`CoordinateTransformer` 用橢球。台灣（緯度 ~23.9°）真實 `m_lat ≈ 110 700` → 約 **0.1%**。
影響：3 km 條帶差約 3 m、24.5 km 總航程差約 25 m、makespan 差約 1 s。
不影響飛安（遠小於 `d_sep_m` 100 m 與 GPS 精度），但**既有 golden values 不可用來驗收 hv 分支**（§17.4）。
---

## 10. 載具設定：兩個構型的參數怎麼填

### 10.1 平台常數來自哪裡（不要自己打）

`hv_dccpp.platform.PLATFORM_PIONEER`（A）與 `PLATFORM_HERO`（B）的欄位值逐一取自 `docs/平台參數表.md` 表 1：

| 表 1 鍵 | A（Pioneer PNP） | B（HERO PNP） | 作用 |
|---|---|---|---|
| `platform_type` | `lift_and_cruise_vtol_4plus1` | 同左 | 4 升力旋翼 + 後推固定翼 |
| `reference_auw_kg` / `reference_payload_kg` | 24.6 / 5.0 | 6.65 / 0.6 | `SortieEnergy` 預設 |
| `cruise_airspeed_mps` | **21.5** | **19.0** | 巡航空速（取代 UI 的 `speed`） |
| `turn_radius_planning_m` | **200** | **120** | 規劃轉彎半徑（取代 UI 的 `turn_radius`） |
| `planning_climb_angle_deg` / `planning_descent_angle_deg` | 3.0 / 4.0 | 3.0 / 4.0 | 高度包絡坡度上下限 |
| `min_fw_operation_agl_m` | **120** | **100** | ⚠ 作業高度下限（UI 預設 100 m 會被夾） |
| `max_bank_angle_deg` | 20 | 25 | `r_eff` 的風修正 |
| `vtol_site_clear_radius_m` / `obstacle_horizontal_buffer_m` | 8 / 30 | 6 / 25 | 起降點淨空與障礙緩衝 |
| `reserve_fraction` / `reference_endurance_min` | 0.30 / 176 | 0.30 / 136 | 能量模型 |
| `wind_resistance_public` | Beaufort 5 | Beaufort 5 | 風警告門檻（非放行門檻） |
| `c2_link_range_km` / `terrain_clearance_m` | **待填入** / **待任務定義** | 同左 | ⚠ 必須由任務給（否則 NO_GO） |
| `alternate_landing_sites_required` | true | true | ⚠ 必須給 `alternates` |

實測 `spec.r_eff(W)`：A 無風 200.0 → 8 m/s 風 **244.7 m**；B 120.0 → **151.5 m**。
這正是 AeroPlan 全隊共用單一 `_R = max(turn_radius)`（`swarm_coordinator.py:1184`）丟失的資訊——
用 200 m 規劃 B 過度保守，用 120 m 規劃 A 則直接不可飛。

### 10.2 與 `REPO/config/vehicle_profiles.yaml` 的 `vtol` 區段對應

`:343 vtol.generic_vtol` 是 stub：`mass_kg: 3.0`（與 24.6 / 6.65 差一個量級）、`multirotor_mode`/`fixed_wing_mode`
分離、`transition_time_s: 5.0`、`max_flight_time_min` 全樹無人讀。**不要**把 hv 的平台常數搬進這個 YAML：
(a) 該結構表達不了 `PlatformSpec` 的 40+ 欄位；(b) 它只被 `REPO/core/base/fixed_wing_constraints.py:517-536
from_yaml_profile` 讀，而 `FixedWingConstraints` 用 `__slots__` 不能 monkey-patch；
(c) 數值一旦分叉，`docs/平台參數表.md` 就不再是單一權威來源。

**建議**只加**指標性**條目給 UI 顯示，實際數值一律由 hv 提供：

```yaml
vtol:
  # ── HV-DCCPP 構型（實際參數由 hv_dccpp.platform 提供，見 docs/平台參數表.md）──
  pioneer_pnp:
    name: "Pioneer PNP (HV-DCCPP 構型 A)"
    type: "vtol"
    hv_dccpp_class: "A"        # ← 給 _hv_class_of 讀
    payload: "lidar"
    mass_kg: 24.6              # 以下僅供 UI 顯示與地圖圖示，規劃一律以 PLATFORM_PIONEER 為準
    cruise_speed_mps: 21.5
    turn_radius_m: 200.0
    min_operation_agl_m: 120.0
  hero_pnp:
    name: "HERO PNP (HV-DCCPP 構型 B)"
    type: "vtol"
    hv_dccpp_class: "B"
    payload: "rgb"
    mass_kg: 6.65
    cruise_speed_mps: 19.0
    turn_radius_m: 120.0
    min_operation_agl_m: 100.0
```

（`REPO/config/schemas.py:59 FlightParameters` 的 `vehicle_type ∈ {multirotor, fixed_wing, vtol}` 不用改，
A/B 是 `vtol` 之下的細分。）

### 10.3 `class_of` 回呼（**整合的關鍵**）

`default_class_of(drone)` 判定順序（`test_default_class_of` 逐條釘住）：`hv_class`／`class` 鍵 →
`platform` 含 `pioneer`→A／`hero`→B → `payload`／`sensor`／`sensor_kind` 含 `lidar`→A／`rgb`|`cam`→B →
`name` 中「後面緊接數字」的 A/B（`"UAV-B2"`→B；`"UAV-1"` 不匹配）→ 都沒有 → **`"A"`**。

⚠ **最後一條是陷阱**。`REPO/ui/main_window.py:4722-4744` 的 drone dict 只有 `drone_id`／`name`（`f'UAV-{i+1}'`）／
`position`／`speed`／`heading`／`turn_radius`／`vehicle_type`／`fov_width`——**沒有任何構型線索**。
`default_class_of` 會把整隊當成 Pioneer，於是 HERO 機被以 24.6 kg AUW、200 m 轉彎半徑、120 m 最低高度規劃，
**結果完全錯誤且不會報錯**。所以實務整合一定要自己提供 `class_of`：

```python
# 1（推薦）UI 逐機設定：由 §6 的兩個 spinbox 決定，前 n_a 台是 A
def _hv_class_of(self, drone: dict) -> str:
    return self._hv_classes[int(drone['drone_id'])]      # {1:'A', 2:'A', 3:'B', ...}

# 2 組 drone dict 時就寫進去，讓 default_class_of 走第 1 條規則（class_of 可留 None）
drones.append({..., 'hv_class': 'A' if i < n_a else 'B'})

# 3 從 vehicle_profiles.yaml 的 hv_dccpp_class 查（配合 §10.2）
def _hv_class_of(self, drone: dict) -> str:
    cls = str(self.vehicle_profiles.get(drone.get('profile_key', ''), {}).get('hv_dccpp_class', '')).upper()
    return cls if cls in ('A', 'B') else 'A'
```

回傳 `'A'`/`'B'` 以外的值 → `AeroPlanAdapterError`（`test_invalid_inputs`）。
檢查方法：`coord.id_map` 應是 `{1: 'A1', 2: 'B2'}`，不是 `{1: 'A1', 2: 'A2'}`。

### 10.4 感測器

未給時：A → `(LidarHardware(),)`（`fov_full_deg` 120、`fov_usable_deg` 70、`point_rate_hz` 600 000、`max_range_m` 450）；
B → `(CameraHardware(),)`（`focal_length_mm` 16.0、`sensor_width_mm` 23.5、`pixel_pitch_um` 3.9）。
這些是**示例值**（`is_default_example=True`），preflight 記 `sensor_spec_default_example(A)`／`(B)`。

```python
from hv_dccpp.platform import CameraHardware, LidarHardware, TrapezoidFovSpec
coord = HvDccppCoordinator(class_of=my_class_of,
    sensor_a=(LidarHardware(fov_full_deg=120.0, fov_usable_deg=70.0, point_rate_hz=600_000.0,
                            return_efficiency=0.5, max_range_m=450.0, range_margin=0.9,
                            is_default_example=False),),
    sensor_b=(CameraHardware(focal_length_mm=16.0, sensor_width_mm=23.5, sensor_height_mm=15.6,
                             pixel_pitch_um=3.9, max_shutter_rate_hz=1.0, is_default_example=False),))

# 論文梯形 FOV（Eq.(2)(3)）——cfg 必須是 fov_model="trapezoid"
from hv_dccpp.mission import PlannerConfig as HvPlannerConfig
coord = HvDccppCoordinator(class_of=my_class_of,
    cfg=HvPlannerConfig(fov_model="trapezoid", dem_step_m=25.0),
    sensor_a=(TrapezoidFovSpec(kind="lidar", mount_angle_deg=15.0, v_fov_deg=45.0, h_fov_deg=77.0),),
    sensor_b=(TrapezoidFovSpec(kind="rgb",   mount_angle_deg=15.0, v_fov_deg=45.0, h_fov_deg=77.0),))
```

### 10.5 酬載需求——最容易踩的雷

`ctx.sensors` 的鍵是 `(構型, req_id)`。**某構型若找不到「同 kind」的需求，就完全沒有 `Sensor`，
該構型完全無法被分配，而且不報錯**——症狀是「異構機隊只有一種構型在飛」。
`_default_payload_reqs` 在 A（LiDAR）與 B（相機）並存時自動產生**兩個**需求：`"default"`（主種類）與
`"default_rgb"`／`"default_lidar"`（`test_payload_reqs_cover_every_sensor_kind` 斷言 kind 集合為 `{"lidar","rgb"}`）。
自己指定時**務必兩種 kind 都給**：

```python
from hv_dccpp.mission import PayloadRequirement
coord = HvDccppCoordinator(class_of=my_class_of, payload_reqs={
    "lidar_hi": PayloadRequirement("lidar_hi", "lidar", density_pt_m2=25.0, side_overlap=0.25,
                                   forward_overlap=0.70, h_target_agl_m=160.0),
    "rgb_hi":   PayloadRequirement("rgb_hi", "rgb", gsd_required_m=0.03, side_overlap=0.30,
                                   forward_overlap=0.70, h_target_agl_m=140.0)})
# 區域指定用哪個需求：areas 元素加 payload_req_id，或 sensor_req = 'lidar'/'rgb'/'any'
areas = [{'area_id': 1, 'polygon': ring1, 'priority': 1.0, 'payload_req_id': 'lidar_hi'},
         {'area_id': 2, 'polygon': ring2, 'priority': 0.6, 'sensor_req': 'rgb'}]
```

### 10.6 能量

```python
coord = HvDccppCoordinator(class_of=my_class_of, energy={
    'A': {'auw_kg': 24.6, 'payload_kg': 5.0, 'available_wh': 1350.0, 'pack_nominal_wh': 1500.0},
    'B': {'auw_kg': 6.65, 'payload_kg': 0.6, 'available_wh': 380.0,  'pack_nominal_wh': 420.0}})
```

未給時用 `DEFAULT_ENERGY_A`／`DEFAULT_ENERGY_B`（就是上面的值）。這些 dict 直接展開成 `SortieEnergy(uav_id, **e)`，
可用鍵即 `SortieEnergy` 的欄位：`auw_kg`／`payload_kg`／`available_wh`／`available_mah`／`pack_voltage_nom_v`／
`pack_nominal_wh`／`soc_fraction`／`battery_temp_c`（`uav_id` 由轉接層填，你給了會被移除）。
§3 實測：A1 用 310.8 Wh／餘裕 589.2 Wh；B2 用 111.3 Wh／餘裕 142.7 Wh。

---

## 11. 地形、禁飛區、Home 與備降點

### 11.1 地形：`terrain_func` → `DemGrid`

**`dem_path` 不會被讀取**（刻意不相依 rasterio）。AeroPlan 已在 `REPO/mission/swarm_coordinator.py:1064-1075`
把 `dem_path` 轉成回呼（`:1070 terrain_func = dem_mgr.get_elevation`，即 `REPO/sensors/dem_terrain.py:175`），
所以方式 A 的插入點（`:1076`）已經拿到它。方式 B 要自己給：

```python
    def _terrain_func_or_none(self):
        """回傳 f(lat, lon) -> AMSL m，或 None（平地）。"""
        mgr = getattr(self, '_dem_manager', None)      # 你載入 DEM 時存的 DEMTerrainManager
        return mgr.get_elevation if mgr is not None else None
```

回呼回 `None`/`nan` 的節點由 `terrain_func_to_dem` 以 `base_msl_m` 填補；回呼丟例外也一樣（不中斷規劃）。
`test_terrain_func_to_dem_sampled` 釘住這三種行為。

**取樣成本**：`terrain_func_to_dem` 是 **Python 雙層迴圈**逐節點呼叫回呼；範圍 = 全部幾何（區域 + home + 備降點）
外接矩形 ⊕ `dem_margin_m`。§3 情境 + `dem_margin_m=3000`、`step_m=50` → 約 200 × 180 = 36 000 次呼叫，約 1–3 s，
**發生在 `progress` 的 0.02 → 0.15 之間**。調整：加大 `dem_step_m`（50→100 → 節點 1/4）；`max_nodes`（預設 4 000 000）
超過時**自動加倍 `step_m`** 不丟例外（`test_terrain_func_to_dem_node_cap`）；或自己先建 `DemGrid` 快取重用。

#### ⚠ `dem_margin_m` 預設值太小（實測發現，務必調整）

`HvDccppCoordinator` 的 `dem_margin_m` 預設 **1500.0 m**，但 `hv_dccpp.context.TERRAIN_MARGIN_M` 是 **3000.0 m**
——`build_context` 會把 ENU 地形網格鋪到「幾何外接矩形 ⊕ 3000 m」，超出 `DemGrid` 範圍的節點取樣為 **`nan`**
（`hv_dccpp.geo.DemGrid.sample_msl` 明文「範圍外 nan」）。本機實測（§3 情境）：

```text
dem_margin_m 1500.0 → ENU 地形網格 nan 比例 0.550   （55%！）
dem_margin_m 3000.0 → ENU 地形網格 nan 比例 0.005
```

平地情境下 nan 落在從未被查詢的外圈（兩者 makespan 都是 1479.4 s，看不出差別），
**但在真實起伏地形上，只要航跡或走廊落進那一圈就會拿到 nan**。👉 **一律傳 `dem_margin_m=3000.0`**。

自己建 `DemGrid`（跨規劃重用）：

```python
from hv_dccpp.integration.aeroplan_adapter import terrain_func_to_dem
dem = terrain_func_to_dem(dem_mgr.get_elevation,
                          bounds_latlon=(lat_min, lon_min, lat_max, lon_max),   # 已含 ≥ 3000 m 裕度
                          step_m=50.0, base_msl_m=0.0,
                          sigma_z_m=2.0,      # DEM 1σ 高程誤差 → h_clr ≥ 3σ+30（D11）
                          is_dsm=False,       # True = 已含樹/建物 → 障礙不再燒入（D24）
                          max_nodes=4_000_000)
```

`sigma_z_m` 有實質影響：`terrain_clearance_m=None` 時取 `max(36, 3σ+30)`，σ=2 → 36 m。

### 11.2 禁飛區：`FenceZoneRegistry` → `NoFlyZone`

```python
    def _nfz_zones(self):
        from mission.fence_zone import FenceZoneRegistry, ZoneCategory   # fence_zone.py:181, :220
        reg = FenceZoneRegistry.instance()
        return reg.by_category(ZoneCategory.NFZ) + reg.by_category(ZoneCategory.THREAT)
```

然後 `HvDccppCoordinator(fence_zones=self._nfz_zones())`，或直接
`fence_zones_to_nofly(zones, alt_ref="msl", home_msl_m=0.0, circle_segments=16, include_categories=("NFZ","THREAT"))`。

* **duck typing，不 import AeroPlan**：任何具 `category`/`shape`/`vertices`/`center`/`radius_m`/`alt_min_m`/`alt_max_m`
  的物件都行；有 `polygon_vertices(circle_segments)` 方法者**優先使用**（`fence_zone.py:87`）。
* **`GEOFENCE` 預設跳過**：它是 **inclusion**（UAV 必須留在**內**），語意與 `NoFlyZone` 相反。
* **高度基準未定**：`fence_zone.py:61-63` 註解寫「AMSL 或 AGL 依專案慣例」。`NoFlyZone` 一律 **MSL**（規格 §6.1、D42）。
  若貴專案把 `alt_*` 當 AGL，傳 `alt_ref="agl", home_msl_m=<home 地面 MSL>`
  （`test_fence_agl_reference_shifts_by_home_msl`：`alt_max_m=120` + `home_msl_m=250` → `z_ceiling_msl_m=370`）。
* 頂點 < 3 或圓形缺 `center`/`radius_m` → **略過**（不丟例外）。`zone_id` 取 `id`→`name`→`NFZ{i+1}`，重複時加 `#{i+1}`；
  `alt_max_m <= alt_min_m` 的無效區間 → 天花板視為 `None`。

### 11.3 Home 與備降點

AeroPlan 的 Home 是 `REPO/ui/main_window.py:205 self.home_point`（無則取多邊形形心，`:4712-4718`）；
**備降點在 AeroPlan 完全沒有這個概念**，但表 1 `alternate_landing_sites_required = true`。

```python
from hv_dccpp.mission import Site
coord = HvDccppCoordinator(class_of=my_class_of,
    home=self.home_point,               # (lat, lon) 或 Site；None → 取 drones[0]['position']
    alternates=[(23.9400, 120.8800),    # 純座標 → Site(f"ALT{i+1}", kind="alternate")
                Site("ALT_N", 23.9800, 120.9300, kind="alternate",
                     clear_radius_m=10.0, approach_heading_deg=None)])   # None → 逆風進場（D19）
```

⚠ 未給 `alternates` 時轉接層會自動在 home 背向區域形心 400 m 放一點並記 note `alternate_auto_placed(...)`。
這只是為了讓 preflight 不 NO_GO，**那個點不見得能降落**（可能在水裡、在建物上）——務必由操作員在地圖上選。
自動點若通不過站點檢查（坡度、淨空、障礙緩衝），preflight 會給 `no_usable_alternate(A)` → NO_GO。

### 11.4 Rally 點

AeroPlan 的 rally 只是空 stub（`REPO/mission/geofence_manager.py:467` `"rallyPoints": {"version": 2, "points": []}`）。
`hv_dccpp.ardupilot.export_all` 會為每機產生 `<uav_id>_rally.txt`（home + 各備降點）：

```python
import hv_dccpp as hv
summary = hv.export_all(coord.last_plan, coord.last_ctx, coord.cfg, export_dir,
                        alt_frame="relative", camera_trigger=True)
# → A1.waypoints A1_rally.txt A1_fence.txt A1.param A1.plan（B2 同）+ mission_summary.json
```

要送上飛控，AeroPlan 側沒有現成路徑（`sitl_link.py` 有 `upload_fence_zones :268`／`upload_fence :297`／
`clear_fence :304`，**沒有** rally）。兩個選擇：(a) 操作員用 Mission Planner 載入 `<uav>_rally.txt`；
(b) 走 `hv_dccpp.mavlink_link.FleetLink` 自建連線上傳（`mission_type=2`）。

---

## 12. 匯出與上傳

### 12.1 QGC WPL 110

```python
per_drone = coord.qgc_waypoints(key="drone_id")      # {'1': [...], '2': [...]}
# 或 plan_to_aeroplan_waypoints(coord.last_plan, coord.last_ctx, coord.cfg,
#        header=True, camera_trigger=True, id_map=coord.id_map, key="uav_id") → {'A1': [...], 'B2': [...]}
import os
for name, lines in per_drone.items():
    with open(os.path.join(export_dir, f"UAV-{name}.waypoints"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
```

格式（`qgc_wpl_lines`）與 `REPO/utils/file_io.py:222 create_waypoint_line` **逐位元相同**：12 欄 **tab** 分隔、
`seq current frame command p1..p4 lat lon alt autocontinue`、`lat`/`lon` 6 位小數、`alt` 2 位小數；
`frame`：NAV = **3**（`GLOBAL_RELATIVE_ALT`）、`DO_*` = **2**（`MISSION`）、home 列 = **0**。
`test_qgc_lines_format` 逐欄釘住 → `REPO/utils/file_io.py:129/185` 的讀取器可原樣解析、`main_window.py:5213` 流程可沿用。
`camera_trigger=True` 會插入 `DO_SET_CAM_TRIGG_DIST`(206) 與 LiDAR 的 `DO_SET_SERVO`，放在條帶起／訖 NAV 之**後**
（ArduPilot 在「抵達前一個 NAV」就執行 DO_，放前面會變成「進入條帶前開、整條關」）。

### 12.2 rally / fence / .param / .plan

```python
summary = hv.export_all(coord.last_plan, coord.last_ctx, coord.cfg, export_dir,
                        alt_frame="relative",     # ← 對應 frame 3；不要改成 "amsl"
                        camera_trigger=True,
                        wp_radius_m=None,         # None → min(R_eff/2, 30)（規格 §8.5）
                        write_plan_json=True,     # 產生 <uav>.plan（QGC JSON）
                        takeoff_delay="operator")
```

| `takeoff_delay` | 行為 | 何時用 |
|---|---|---|
| `"operator"`（預設） | 任務檔**不含**延遲項；`t0 + Δt` 寫進 `.waypoints` 檔頭註解與 `mission_summary.json` 的 `takeoff` 區塊。**地面站必須依 `t_takeoff_s` 逐機下 MISSION_START** | 用 AeroPlan 的 SITL 上傳流程時 |
| `"nav_delay"` | 在 `NAV_VTOL_TAKEOFF` 之前插入 `MAV_CMD_NAV_DELAY`(93)，任務自帶錯開 | 同時 AUTO 所有機時。⚠ **不可**再由地面站延後 `MISSION_START`（會重複延遲） |

⚠ 用 `"operator"` 卻同時 AUTO 全部機 → D46 的 60 s 錯開與所有去衝突延遲**全部失效**，4-D 分離結果不再成立。

### 12.3 SITL 上傳（8-tuple）

```python
tuples_by_drone = coord.upload_tuples(
    is_vtol=True,                    # 兩構型都是 4+1 複合翼，正常不改
    cruise_speed_mps=None,           # None → 逐機取 spec.cruise_airspeed_mps（A 21.5、B 19.0）
    turn_radius_m=None,              # None → 逐機取 spec.r_eff(W)
    vtol_transition_alt_m=100.0)     # AeroPlan 由 parameter_panel.vtol_transition_alt_spin 取得（:6030）
for link in self._sitl_links:        # 派送比照 REPO/ui/main_window.py:6188-6210
    sysid = int(getattr(link, 'sysid_label', 1))
    wps = tuples_by_drone.get(sysid)
    if wps:
        link.upload_mission(wps)     # REPO/mission/sitl_link.py:264
```

`sitl_link.py:1214 _send_mission` 接受 3-tuple 或 8-tuple（`:1217-1219`）；協定 `mission_clear_all` → `mission_count`
→ per-request `mission_item_int`（lat/lon ×1e7）→ ACK（`:1250-1290`），第 0 項複製為 Home（`:1237-1245`），
frame 選擇在 `:1270-1274`。產生的 VTOL 序列：

```
DO_CHANGE_SPEED(178, p1=0=空速, p2=cruise, p3=-1=不改油門)
NAV_VTOL_TAKEOFF(84) @ vtol_transition_alt → DO_VTOL_TRANSITION(3000, p1=4=MAV_VTOL_STATE_FW)
NAV_WAYPOINT(16) × N（p2 = turn_radius）
NAV_WAYPOINT(16) @ vtol_transition_alt → DO_VTOL_TRANSITION(3000, p1=3=MC) → NAV_VTOL_LAND(85)（p4 = 落地航向）
```

`DO_*` 項的 lat/lon/alt 全為 `0.0`（`sitl_link.py:1270-1274` 對這些 cmd 用 `MAV_FRAME_MISSION`；
`test_upload_tuples_frame_selection` 釘住）。`is_vtol=False` 退化為
`DO_CHANGE_SPEED → NAV_TAKEOFF(22, p1=10° pitch) → NAV_WAYPOINT × N → NAV_LAND(21)`。
所有指令碼都在 `REPO/mission/mission_validator.py:38-51` 白名單內（16/21/22/84/85/178/3000），
`test_upload_tuples_shape_and_commands` 以 `pymavlink.mavutil.mavlink` 常數（不手寫數字）逐一比對。

#### ⚠ 與 AeroPlan 現行實作的唯一差異：起飛指令 bug

`REPO/ui/main_window.py:6115-6116`：

```python
                        if seg_name == 'TAKEOFF':
                            if idx == 0:
                                continue                # ← 第 0 個 TAKEOFF 航點直接跳過
                            if not takeoff_emitted:
                                ... 輸出 NAV_VTOL_TAKEOFF ...
```

若路徑**只有一個** `TAKEOFF` 航點——**正是 hv 縮點後的情形**（`NAV_VTOL_TAKEOFF` 只需一點）——`idx == 0` 被
`continue` 掉，`takeoff_emitted` 永遠 `False`，**整份任務沒有起飛指令**，飛控會直接從第一個 `NAV_WAYPOINT` 開始。
`to_upload_tuples` 在 `idx == 0` 且尚未輸出起飛指令時**照樣輸出** `NAV_VTOL_TAKEOFF` + `DO_VTOL_TRANSITION(4)`，
其餘與 `:6045-6159` 逐項相同。

👉 **走 hv 分支時 SITL 上傳請改用 `coord.upload_tuples()`**。在 `REPO/ui/main_window.py:6041-6043` 加分支：

```python
                ap = self._dccpp_result.get('assembled_paths') or {}
                _hv = getattr(self, '_hv_coordinator', None)
                if _hv is not None and _hv.last_result is self._dccpp_result:
                    per_uav_wps = _hv.upload_tuples(vtol_transition_alt_m=vtol_transition_alt)
                else:
                    per_uav_wps = {uid: _build_wps_for_path(p) for uid, p in sorted_paths}
```

---

## 13. 動態重規劃

```python
"""在 MainWindow 內：定時（或由 FleetRegistry 的 signal 觸發）檢查是否需要重規劃。"""
import hv_dccpp as hv
from hv_dccpp.integration.aeroplan_adapter import from_mission_plan


def _hv_dynamic_tick(self):
    coord = getattr(self, '_hv_coordinator', None)
    if coord is None or coord.last_plan is None:
        return
    from mission.fleet_registry import FleetRegistry
    frames = list(FleetRegistry.instance().snapshot().values())      # fleet_registry.py:106
    if not frames:
        return
    events, tels = coord.events_from_telemetry(
        frames, t0_epoch_s=self._hv_t0_epoch,     # 任務起始 epoch 秒（按下 auto_start 時記下）
        link_timeout_s=5.0,
        last_link_ok=self._hv_last_link_ok,       # {uav_id: 上一輪 link_ok}；首輪可傳 None
        return_telemetry=True)
    self._hv_last_link_ok = {uid: t.link_ok for uid, t in tels.items()}
    if not events:
        return
    if getattr(self, '_hv_cache', None) is None:  # plan_mission 不回傳 CostCache，需重建
        self._hv_cache = hv.cache_from_plan(coord.last_plan, coord.last_ctx)

    for ev in events:
        res = hv.replan(coord.last_plan, ev, tels, coord.last_ctx, self._hv_cache, coord.cfg, [])
        logger.info(f"[HV-DCCPP] {type(ev).__name__} → changed={res.changed_uavs} "
                    f"{res.timing_ms.get('total', 0.0):.0f} ms notes={res.notes[:2]}")
        if res.placeholder_used:
            logger.warning("[HV-DCCPP] 重規劃逾時（> 2 s），已下發佔位航點")   # ← 必須告知操作員
        coord.last_plan, self._hv_cache = res.plan, None          # 計畫變了 → cache 失效
        new_result = from_mission_plan(res.plan, coord.last_ctx, areas=self._hv_areas,
                                       drones=self._hv_drones, id_map=coord.id_map, reduce=True)
        coord.last_result = new_result
        self._dccpp_result = new_result                            # 同 main_window.py:5014
        self._display_dccpp_result(new_result)
        rev = {v: k for k, v in coord.id_map.items()}               # {'A1': 1, 'B2': 2}
        changed_ids = {rev[u] for u in res.changed_uavs if u in rev}
        tuples_by_drone = coord.upload_tuples(vtol_transition_alt_m=100.0)
        for link in self._sitl_links:                              # ★ 只重傳有變的機
            sysid = int(getattr(link, 'sysid_label', 1))
            if sysid in changed_ids and sysid in tuples_by_drone:
                link.upload_mission(tuples_by_drone[sysid])         # sitl_link.py:264
```

### 13.1 哪些事件會改哪些機

| 事件 | `telemetry_frames_to_events` 會產生？ | 影響 |
|---|---|---|
| `EnergyUpdate` | ✅ 電量偏差 > `cfg.delta_1 × E_nom`（`delta_1` 預設 0.05） | 該機 + 可能重新配置給其他機 |
| `LinkLoss` | ✅ `now − last_update > link_timeout_s`（預設 5 s） | 該機轉為幽靈航跡（`ReplanResult.ghosts`），其他機需避開 |
| `LinkRestored` | ✅ 上一輪失聯、本輪正常 | 該機重新納入 |
| `UAVExit` / `WindUpdate` | ❌ `TelemetryFrame` 無 `landed_state` / 風速欄位 | 需自建事件（§13.2） |
| `NewArea` / `TargetCue` / `PriorityChange` / `WeatherAbort` | ❌ 由操作員觸發 | UI 建構事件物件後直接 `hv.replan(...)` |

實測（§3 情境，A1 電量 25%、B2 電量 70%）：

```text
events: ['EnergyUpdate/A1', 'EnergyUpdate/B2']
EnergyUpdate  changed=['A1','B2']  167.8 ms  placeholder=False
   notes=['kappa(A1: 1.000 -> 1.500, delta=0.722)', 'divert_conservative(A1: delta=0.722 > delta_2)']
   new makespan 2248.5 s  航點 {1: 29, 2: 91}
EnergyUpdate  changed=['B2','A1']  167.6 ms   new makespan 1977.4 s  航點 {1: 129, 2: 19}
```

**重規劃 ≈ 168 ms**（`examples/run_dynamic_events.py` 各事件 19–192 ms；mock 閉環 185 ms）。
仍**不建議在 GUI 執行緒做**——丟進 §7 的 worker，或至少確保 tick 間隔 ≥ 1 s。

### 13.2 為什麼只有三種事件

`REPO/mission/sitl_link.py:30-62` 的 `TelemetryFrame` **沒有** `landed_state`（→ 無 `UAVExit(landed_unexpected)`）、
**沒有** `mission_seq`（→ 條帶進度只能用計畫時間軸估計，誤差比 `MISSION_CURRENT` 對照大）、**沒有**風速（→ 無 `WindUpdate`）。
要完整規則就改用 `hv_dccpp.mavlink_link.FleetLink` 自建連線（訂閱 `EXTENDED_SYS_STATE`/`MISSION_CURRENT`/`WIND`）：
`fleet = hv.FleetLink({'A1': 'udpin:0.0.0.0:14550', 'B2': 'udpin:0.0.0.0:14560'})`。
⚠ 這會與 AeroPlan 的 `SITLLink` **搶同一個 UDP 埠**——要嘛全走 AeroPlan 的鏈路（接受三種事件的限制），
要嘛全走 `FleetLink`（放棄 AeroPlan 的 HUD）。**不要兩者並行。**
⚠ 重規劃後驗證要傳 `t_sync_s`：`hv.validate(res.plan, ctx, cache, cfg, t_sync_s=res.t_sync_s)`（R-12），
否則 §13.3 的 `σ_t = k_t (t − t_sync)` 會被灌上 `k_t·t_now`，分離檢查誤報。

---

## 14. SITL 驗證流程

### 14.1 用 AeroPlan 自己的 `SITLLauncher`

`REPO/mission/sitl_launcher.py:399 SITLLauncher`；`tcp_port_for(i) = 5760 + 10·i`（`:416`）；
`default_instance_configs(n)`：`sysid = i+1`、`embedded_port = 14550 + 10·i`（`:431`）；`start(...)`（`:470`）／
`start_multi(...)`（`:693`）；VTOL → `ArduPlane.exe`（`:174-178`）。
GUI 的 `REPO/ui/main_window.py:6237 on_sitl_launch` 會**自動由 `assembled_paths` 推導各機 spawn home**（`:6330-6356`）
並覆蓋數量（`:6373-6379`）——所以**先規劃、再啟動 SITL**，各機就會生在自己的起飛點上。

步驟：① UI 畫測區、設 Home、選 `vehicle_type = vtol`、開啟 HV-DCCPP；② 按規劃、等 worker 完成；
③ `on_sitl_launch` 起 N 台 ArduPlane SITL；④ 等 `SITLLink.connected` signal（`Q_ENABLE=1` 由 `sitl_link.py:468` 確認，
因為 QuadPlane 心跳回報 `MAV_TYPE_FIXED_WING`，`:195-199` 的 `vehicle_hint` 就是為此存在）；
⑤ 上傳用 §12.3 的 `coord.upload_tuples()`（**不要**用 `_build_wps_for_path`）；
⑥ `link.mission_start()`（`:310`）或 `auto_start()`（`:312`）——⚠ `takeoff_delay="operator"` 時**必須依
`mission_summary.json` 的 `takeoff` 區塊逐機延遲**，否則錯開起飛與 4-D 去衝突全部失效（§12.2）；
⑦ 觀察 HUD、地圖航跡、`FleetRegistry` 遙測，用 §13 的 tick 檢查事件。

**驗收點**：兩台高度**不同**（分層，`dh_sep_m` 50 m；實測 A 169.9 m、B 160.4 m 相對 home）；
水平距離全程 ≥ `d_sep_m`（100 m）；每台第一個任務項是 `DO_CHANGE_SPEED`、第二個是 `NAV_VTOL_TAKEOFF`
（若第一個 NAV 就是 `NAV_WAYPOINT`，代表還在用 `_build_wps_for_path`，§12.3）；
落地總時間接近 `total_makespan`（本情境 1479.4 s，需按 SITL 加速倍率換算）。

### 14.2 沒有 SITL 時：`tools/mock_autopilot.py`

`hv_dccpp` 自帶以 pymavlink 寫的最小 QuadPlane 模擬器，支援心跳、任務／圍籬／集結點上傳與下載、
`MISSION_CLEAR_ALL`、`PARAM_SET`、`COMMAND_LONG`（arm／`MISSION_START`／`DO_SET_MODE`），
以及 AUTO 模式沿 NAV 航點的簡化飛行模型（發 `GLOBAL_POSITION_INT`／`MISSION_CURRENT`／`BATTERY_STATUS`／
`WIND`／`EXTENDED_SYS_STATE`／`MISSION_ITEM_REACHED`；`NAV_DELAY(93)` 依 `param1` 原地等待）。

```powershell
# 機 A（構型 A 的實際電量與功率）
python "C:/Users/joy46/OneDrive/文件/vtol演篹法/tools/mock_autopilot.py" `
    --listen udpin:127.0.0.1:14550 --home 23.95,120.90,0,0 --speed 21.5 --time-scale 20 `
    --battery-wh 1350 --cruise-power-w 330 --hover-power-w 2600 --reserve-frac 0.30 `
    --dump received_mission_A1.json
# 機 B
python tools/mock_autopilot.py --listen udpin:127.0.0.1:14560 --home 23.95,120.90,0,0 `
    --speed 19.0 --time-scale 20 --battery-wh 380 --cruise-power-w 95 --hover-power-w 700 `
    --dump received_mission_B2.json
```

然後把 `SITLLink` 的 `conn_str` 指過去（`udpout:127.0.0.1:14550`），或直接跑完整閉環示範
`python examples/run_mock_autopilot_loop.py`（實測：兩機上傳 98 + 54 項、重規劃 185 ms、雙機正常降落，全程 49.5 s）。

⚠ `--battery-wh`／`--cruise-power-w` 的**預設值（800 Wh / 350 W）與任何實際平台都不一致**，
會讓 `detect_events` 一直看到能量偏差。要誠實的閉環請用上面的實際值，或餵計畫的能量剖面
（`--energy-profile A1_energy.json`；`--energy-error-frac 0.1` 可蓄意多耗 10% 來觸發 `EnergyUpdate`）。
`--dump received_mission_A1.json` 會把飛控**實際收到**的任務項寫成 JSON——
這是驗證「8-tuple → `mission_item_int` → 飛控」整條路沒走樣的最直接方法。
---

## 15. 驗證與除錯

### 15.1 `preflight` 的三種狀態

`GO`（無錯誤無警告）／`GO_WITH_WARNINGS`（**最常見**，警告進 `on_report(payload)['warnings']` 與
`coord.notes`，前綴 `preflight:`）／`NO_GO`（`raise_on_no_go=True`（預設）→ 丟 `AeroPlanAdapterError`，
`.errors` 帶清單；`False` → 回空結果並記 note `NO_GO(回傳空結果)`）。
`on_report` 恰呼叫一次（`test_progress_and_report_callbacks`），payload =
`{'status','errors','warnings','notes','makespan_s','timing_ms','uncovered'}`。
**`uncovered` 非空代表有區域沒被完全覆蓋——這是最該紅字顯示給操作員的一項。**

### 15.2 常見錯誤訊息對照表

| # | 訊息（子字串） | 層級 | 意思 / 怎麼修 |
|---|---|---|---|
| 1 | `c2_link_unresolved(A)`：缺鍵 `c2.range_km[A]` … | 🔴 NO_GO | C2 半徑未定（表 1 `c2_link_range_km` =「待填入」）→ 傳 `c2_range_km=25.0`；UI 應加此欄位（§8.2） |
| 2 | `terrain_clearance_unresolved`：缺鍵 `airspace.terrain_clearance_m` … | 🔴 NO_GO | 對地淨空未定（D11 必填；表 1 為「待任務定義」）→ 傳 `terrain_clearance_m=50.0`，或留 `None` 自動取 `max(36, 3σ+30)` |
| 3 | `alternates_missing(A,B)`：缺鍵 `alternates` | 🔴 NO_GO | 無備降點（表 1 要求 true）→ 傳 `alternates=[(lat, lon), …]`。未給時自動放一點並記 `alternate_auto_placed`，**那點不見得能降落**（§11.3） |
| 4 | `no_usable_alternate(A)` | 🔴 NO_GO | 備降點坡度／淨空半徑／障礙緩衝不合格 → 換位置；細節看同時出現的 `site_check(ALT1, A): …` |
| 5 | `A_needs_waiver(Z1: h_ceil … 無豁免，Δh_band=… < …)` | 🟠 | `waiver_max_agl_m=None` 時只有 120 m 天花板，而 A 的下限就是 120 m → **平地沒有高度帶可用**。給 200.0 並確認法規真有豁免 |
| 6 | `coverage_altitude_clamped(100.0->120.0; min_fw_operation_agl=120, ceiling=200)` | 🟡 | UI 預設 100 m 低於 A 的 120 m 下限，已夾住。正常；把 UI 預設改成 ≥ 120（建議 160） |
| 7 | **無錯誤，但 `allocation` 只有一種構型、少一台** | 🔴 **靜默** | `PayloadRequirement` 沒涵蓋該構型的感測 kind → `ctx.sensors[(cls, req_id)]` 不存在 → 該構型完全無法被分配（§10.5）。另一可能：`class_of` 沒給，整隊被當成 A（§10.3） |
| 8 | `energy_unresolved(A1): …` | 🔴 NO_GO | `SortieEnergy` 給了 Wh/mAh 卻無 nominal，或欄位互斥 → 檢查 `energy={'A': {...}}` 的鍵（§10.6） |
| 9 | `validate` 的 `energy_ok=False`，或某機 `energy_margin_wh < 0` | 🟠 | 作業 + 轉場 + RTB + 保留（0.30）超過可用 Wh → 減條帶／降高度／加大 `available_wh`／加機 |
| 10 | `gust_above_public_wind_rating(…)` / `wind_layer_above_go(…)` | 🟠 | 陣風或風層超過表 1 `wind_resistance_public`（Beaufort 5）或放飛門檻。**這是平台能力宣稱，不是自動放行門檻**——由操作員判斷 |
| 11 | `gsd_unattainable(Z1, B: h_max,B(GSD_req)=… < …)` | 🟠 | 相機在該高度達不到要求 GSD（`h_max = GSD·f/p`）→ 降高度或放寬 `gsd_required_m` |
| 12 | `altitude_band_infeasible(Z1, A: <reason>, Δh_band=…)` | 🟠 | 該區對該構型沒有可行高度帶（地形 + 淨空 + 天花板夾死）→ 提高 `waiver_max_agl_m`、降 `terrain_clearance_m`（要有依據）、或該區改派另一構型 |
| 13 | `uncoverable(Z1/A: lidar 12.3%)` | 🟠 | 該區有一部分無法被覆蓋 → 看 `payload['uncovered']`；通常要改機隊組成或放寬約束 |
| 14 | `no_coverable_area` | 🔴 NO_GO | 11–13 的原因全中；先查 `waiver_max_agl_m` 與 `class_of` |
| 15 | `validate` 的 `separation_ok=False` / `conflicts` 非空 | 🟠 | 4-D 去衝突有殘餘（山丘 8 機實測殘餘 4 筆）→ 已知限制（§16.4）；實務上加大 `dt_dep_s` 或減少同時作業機數 |
| 16 | `transition_slope_exceeds(A: … > …)` | 🔴 NO_GO | 起降走廊轉換段坡度超限 → 起降點附近地形太陡，換 home／備降點 |
| 17 | `time_window_disjoint(…)` / `time_window_insufficient(…)` | 🔴 NO_GO | 區域窗與空域窗無交集或窗長不足 |
| 18 | `ModuleNotFoundError: No module named 'pymavlink'` | 🔴 | `pip install pymavlink`（AeroPlan 已列在 requirements，獨立環境可能沒有） |
| 19 | `AeroPlanAdapterError: class_of 須回傳 'A' 或 'B'…` / `areas[0] 之 polygon 至少 3 點…` / `尚未規劃：請先呼叫 plan_coverage_dccpp` | 🔴 | 分別見 §10.3、UI 已在 `main_window.py:4690` 檢查、呼叫順序錯（`test_helper_methods_require_plan`） |
| 20 | `provisional_params(A: a_tr_mps2, battery_nominal_wh, c2_link_range_km, …)` | 🟡 | 表 1 中標為「暫定 placeholder，待實測覆寫」的欄位被使用。**正常，不用修** |
| 21 | `avionics_unvalidated(A: airspeed_sensor_validated=False, battery_monitor_validated=False → k_unc=1.2075, …)` | 🟡 | 表 1 明列這兩項為 `false`；規劃已自動加 **20.75%** 不確定性係數。校正後改 `True` 才會降 |
| 22 | `fov_width_mismatch(cls=A: 224 m @ 160 m AGL vs UI coverage_width_m=160 m)` | 🟡 | **正常**（設計如此，§8 第 3 列） |
| 23 | `dem_path_ignored(C:/dem.tif)` | 🟡 | 本層不讀 raster → 改傳 `terrain_func`（§11.1）。走方式 A 時 `swarm_coordinator.py:1064-1075` 已自動轉好 |

### 15.3 `validate` 的旗標

```python
cache = hv.build_validation_cache(coord.last_plan, coord.last_ctx, budget_only=True)
vrep = hv.validate(coord.last_plan, coord.last_ctx, cache, coord.cfg)
print(vrep.passed, vrep.coverage_ok, vrep.min_sep_h_m, vrep.min_sep_v_m, len(vrep.conflicts))
```

`ValidationReport`（`hv_dccpp/sim.py:103`）：`coverage_ok`（property）／`slope_ok`（坡度未超 3.0°/4.0°）／
`agl_ok`（AGL 在 `[h_clr, 天花板]` 內）／`curvature_ok`（曲率未超 `r_eff`）／`energy_ok`（含 0.30 保留）／
`link_ok`（在 C2 半徑內）／`airspace_ok`（未進 NFZ、未超 AGL 上限）／`time_window_ok`／
`separation_ok`（§13 分離檢查確實跑完；丟例外 → False 且 `passed` 必為 False）／
`integrity_ok`（R-14：每個 `OPERATE` 航點 `sensor_on` ∧ 每條條帶恰好被飛一次）／
`min_sep_h_m`·`min_sep_v_m`（應 ≥ `d_sep_m`／`dh_sep_m`）／`conflicts`／`details`（`{uav_id: [違規字串]}`）。

---

## 16. 效能與限制

### 16.1 實測數字

| 情境 | 規模 | 耗時 | makespan | 備註 |
|---|---|---|---|---|
| §3 / `test_aeroplan_adapter.py` | 1 區 3 km × 1.2 km、**2 機**、7 條帶 | **1.20 s**（`timing_ms.total` 909 ms） | 1479.4 s | 48 項測試 4.71 s |
| `examples/run_mini.py` | 1 區、2 機、8 條帶 | 約 1.0–1.05 s（腳本含繪圖 3.5 s） | 1736 s | 覆蓋 1.000、`validate` **PASS** |
| `examples/run_hills.py` | 3 區山丘、**8 機、107 條帶** | 腳本含繪圖 **42.8 s** | 5552 s | 覆蓋全 1.000；**殘餘 4 筆 4-D 衝突**（`validate` FAIL 僅因此項） |
| `examples/run_paper_table2.py` | 論文 Table 2，3 機 | 腳本含繪圖 **65.1 s** | 3950 s | 679.7／688.9／840.0 km（論文 648／606／591） |
| `examples/run_dynamic_events.py` | 各事件重規劃 | 8.6 s（全腳本） | — | 單次 **19–192 ms** |
| `examples/run_mock_autopilot_loop.py` | mock 閉環兩機 | 49.5 s（全腳本） | — | 上傳 98 + 54 項、重規劃 185 ms |
| 全套測試 | `pytest tests -q` | **105.77 s** | — | **809 passed / 0 failed** |

§3 逐段（ms）：`layering 284.7`、`decompose 222.1`、`assemble_3d 107.8`、`cache 100.4`、`deconflict 68.7`、
`allocate 56.9`、`idp 29.0`、`corridors 24.0`、`preflight 15.7`、`total 909.3`。
最貴的 `layering + decompose` 佔 56%；機數增加時 `allocate`/`idp`/`deconflict` 非線性成長。

### 16.2 UI 凍結對策（詳見 §7）

必用 QThread；0.40→0.90 無中間進度 → 用不定量 `QProgressBar`；`plan_mission` 無取消點（`cancel()` 只在階段邊界生效，
要真正即時取消得用子行程）；重規劃（168 ms）也別放 GUI 執行緒；DEM 取樣（0.02→0.15）大範圍佔數秒 → 先快取 `DemGrid`。

### 16.3 記憶體

| 物件 | 大小 | §3 實測 |
|---|---|---|
| `DemGrid.z_msl`（float32 經緯網格） | `rows × cols × 4 B` | `dem_margin_m=3000`、`step_m=50` → 約 200×180 ≈ 144 KB |
| `ctx.terrain`（ENU 網格，`cfg.dem_step_m`） | `ny × nx × 4 B` | **289 × 537 = 155 193 節點 ≈ 621 KB** |
| `CostCache` | 與條帶數平方相關 | 7 條帶微不足道；107 條帶（山丘）數十 MB |
| 航點 | 每點約 200 B | 每機 387–494 點（縮點前）< 100 KB |

主旋鈕是 `cfg.dem_step_m`：`PlannerConfig()` 預設 **10.0 m**，`HvDccppCoordinator` 預設改為 **25.0 m**（節點 1/6.25）。
10 km × 10 km @10 m = 100 萬節點 ≈ 4 MB（可接受）；50 km 範圍 = 2500 萬節點 ≈ 100 MB → 務必調到 25–50 m。
`terrain_func_to_dem` 的 `max_nodes`（預設 4 000 000 ≈ 16 MB）是防呆上限，超過自動加倍 `step_m`。

### 16.4 已知限制

**hv_dccpp 本身**（摘自 `hv_dccpp/CONTRACT_DEVIATIONS.md`）

1. **山丘情境殘餘 4 筆 4-D 衝突**：需規格 §13.6 步 (iii)（IDP 重排；`deconflict.repair(resequence_hook=…)` 介面已備妥）。
   單機時間平移已被驗證為「補此失彼」，且 8 機共用單一降落走廊時延遲盤旋圈數被推到 39/58 圈而能量不足。
2. **「異構比 all_A 慢 29%」的比較無效**：`sim.baselines` 回報 `all_A = inf`（5 區未覆蓋）、`all_B = inf`（6 區）、
   `legacy = inf`（1 區），只有 hetero 完整覆蓋——那些基線本身不是合法任務。
3. **論文 Table 2 路徑長高於論文**：主因是 `MissionInputs` 只支援單一起飛點，論文假設每機各自的地面站。
4. **平台參數表大量欄位為「暫定 placeholder」**（transition AGL、site clear radius、C2 半徑、terrain clearance…），
   preflight 逐一列在 `provisional_params(...)`；`airspeed_sensor_validated`/`battery_monitor_validated` 皆 `false`
   → 自動加 `k_unc = 1.2075`。

**轉接層**

5. `HvBuiltWaypoint.alt` 是 AGL 相對 home，**起伏地形上 ≠ `agl_m`**（§9.2）；14→5 相位映射**不可逆**（§9.1）。
6. `HvMdtspResult.makespan` 是**秒**（AeroPlan 現行填公尺，§9.4）；`total_makespan` 定義不同，**必然較大**（§9.5）；
   `*_length_m` 是**水平**長度，必然略小於 `UAVPlan.distance_m`（§9.3）。
7. `telemetry_frames_to_events` 只產生 3 種事件（§13.2）；`dem_path` 不讀（§11.1）；`runway_*`/`landing_*` 全忽略（§8）；
   `auto_scan_angle=False`／`coordination_mode`／`collision_avoidance=False`／`enable_altitude=False` **都關不掉**對應功能。
8. `use_native_types` 能以 `setattr` 補兩個長度欄位，但**無法補 `to_latlon_list()` 方法**——用原生型別時下游只能呼叫 `to_latlon()`。
9. **未在真實 PyQt6 環境跑過**（本機未安裝 PyQt6）。§6、§7 的 Qt 程式碼依實際 API 撰寫但**未執行**；
   6 鍵回傳與 duck typing 契約由 48 項測試（含以替身重現原生 `BuiltPath`/`SegmentLabel`）驗證。
10. **`dem_margin_m` 預設 1500 m < `TERRAIN_MARGIN_M` 3000 m** → 地形網格 55% 為 nan（§11.1，本機實測）。**一律傳 3000.0。**

**AeroPlan 側的 11 項阻礙**

| # | 阻礙 | 位置 | 影響 |
|---|---|---|---|
| 1 | 🔴 `fov_width` 從未被讀 | `swarm_coordinator.py:1090` 寫、無人讀 | 異構刈幅無法用既有介面表達 → 轉接層自帶感測器設定 |
| 2 | 🔴 全隊共用單一 `R_min` 與單一高度 | `:1184`、`:1204-1205`；UI 各只有一個 spinbox | 需新 UI 分頁（§6） |
| 3 | 🔴 `'vtol'` 靜默降級為 `MULTIROTOR` | `:1080-1082` | `task_allocator.py:136` 對 QuadPlane 用多旋翼 haversine → 配置會錯。轉接層填 `FIXED_WING` |
| 4 | 🟠 `enable_altitude` 在 live 路徑被忽略 | `:1177-1298` 從未呼叫 `AltitudePlanner` | 高度規劃必須由 hv 全權負責 |
| 5 | 🟠 無續航／能量進入任何規劃器 | YAML 有 `max_flight_time_min` 但無人讀 | 同上 |
| 6 | 🟠 UI 凍結 | `main_window.py:4753-4776` | 必須新增 QThread（§7） |
| 7 | 🟠 名稱衝突 | `PlannerConfig` ×2、`Waypoint` ×3 | 一律用 `Hv` 前綴別名（§17.2） |
| 8 | 🟠 地球半徑／投影不一致約 0.1% | `refactor_audit_2026.md:18-35` | golden values 不可用來驗收（§17.4） |
| 9 | 🟡 `test_dccpp_pipeline.py:16-23` golden 會被打破 | — | 另建 `test_hv_dccpp_adapter.py`，不要改既有 golden |
| 10 | 🟡 geofence 政策宣告但未執行（`@enforce_geofence` 無人用） | — | 建議不套用，改用 hv 自己的 fence 匯出 |
| 11 | 🟡 PyInstaller 與 requirements | `aeroplan_studio.spec:138-143` | §2.3、§2.4 |

另有兩個 hv 分支下**應該關掉**的既有後處理：NFZ 重繞（`main_window.py:4844`）與防撞（`:4852-4870`）——
它們會在已避開 NFZ、已 4-D 分離的路徑上再動一次，而 `*_length_m` 統計不會跟著更新。

---

## 17. 疑難排解 FAQ

**17.1 打包後 `ModuleNotFoundError: hv_dccpp.integration.aeroplan_adapter`** — PEP 562 延遲載入，靜態分析看不到 →
必須 `hiddenimports += collect_submodules('hv_dccpp')`（不是只寫 `'hv_dccpp'`）。還缺就用 `--debug=imports` 跑打包後的 exe。
沒用到 `mavlink_link`/`mqtt_bridge` 可在 `excludes` 排掉縮小體積。

**17.2 名稱衝突 `PlannerConfig` / `Waypoint`** — AeroPlan 有兩個 `PlannerConfig`（`core/base/planner_base.py:70`、
`core/planning/heterogeneous_nfz_planner.py:80`）與三個 `Waypoint`（主要是 `mission/waypoint.py:71`）。
**在 AeroPlan 的檔案裡引用 hv 型別一律加 `Hv` 別名**：

```python
from hv_dccpp import PlannerConfig as HvPlannerConfig
from hv_dccpp.mission import Waypoint as HvWaypoint, MissionPlan as HvMissionPlan
from hv_dccpp.ardupilot import MissionItem as HvMissionItem
```

轉接層自己也這樣做（`aeroplan_adapter.py:132`），輸出型別全叫 `Hv*` 就是為了不撞名。

**17.3 想用 AeroPlan 的真型別而不是 `Hv*`**

```python
import core.dccpp.dccpp_path_builder as pb, core.global_planner.mdtsp_solver as ms, mission.coverage_path as cp
from hv_dccpp.integration.aeroplan_adapter import use_native_types
use_native_types({"SegmentLabel": pb, "BuiltWaypoint": pb, "BuiltPath": pb, "UAVState": ms,
                  "VehicleType": ms, "MDTSPResult": ms, "CoveragePath": cp, "OperationSegment": cp})
```

⚠ 原生 live `BuiltPath`（`dccpp_path_builder.py:65-78`）**沒有** `takeoff_length_m`/`landing_length_m`
（轉接層用 `setattr` 補，該 dataclass 無 `__slots__`），且 **`to_latlon_list()` 補不上來**——
使用原生型別時請確認下游只呼叫 `to_latlon()`。一般情況**不需要**做這件事。

**17.4 既有 golden test 失敗** — 先確認 `use_hv_dccpp` 不是預設 `True`。即使要為 hv 分支寫測試，
**也不要拿 `test_dccpp_pipeline.py:16-23` 的 golden 當基準**（投影差 0.1%、makespan 定義不同 §9.5、
`MDTSPResult.makespan` 單位不同 §9.4）。正確做法是新建 `REPO/tests/test_hv_dccpp_adapter.py`，用自己的 golden 與寬容差：

```python
def test_hv_dccpp_returns_six_keys():
    ad = pytest.importorskip("hv_dccpp.integration.aeroplan_adapter")
    coord = ad.HvDccppCoordinator(class_of=lambda d: 'A' if d['drone_id'] % 2 else 'B')
    res = coord.plan_coverage_dccpp(areas=AREA, drones=DRONES, coverage_width_m=160.0,
                                    overlap_rate=0.25, coverage_altitude=160.0)
    assert set(res) == {'results', 'allocation', 'coverage_paths', 'uav_states',
                        'total_makespan', 'assembled_paths'}
    assert res['total_makespan'] == pytest.approx(1479.4, rel=0.02)   # 自己的 golden，2% 容差
```

（`REPO/pytest.ini` 已設 `--strict-markers`；`slow`/`integration` 都在 markers 清單裡。）

**17.5 matplotlib 後端與 Qt 衝突** — `hv_dccpp.viz` **一被存取就把 matplotlib 後端設為 `Agg`**。
→ **在 AeroPlan 內絕對不要 `import hv_dccpp.viz`，也不要存取 `hv.viz` / `hv.save_all`。**
核心 API 與 `integration.aeroplan_adapter` **都不會**載入 matplotlib（`import hv_dccpp` 約 1.1 s，只拉 numpy/scipy/shapely）。
要出圖請在**另一個行程**跑。

**17.6 logging 慣例** — AeroPlan 兩套：`ui/` 與 `mission/` 用 `from utils.logger import get_logger`；
`core/`（純演算法、不得依賴 Qt）用 `logging.getLogger(__name__)`。所以 `ui/workers/planning_worker.py` 用 `get_logger`。
`hv_dccpp` 自己**不設定任何 handler**，訊息會流進 AeroPlan 的 root logger；要靜音
`logging.getLogger('hv_dccpp').setLevel(logging.WARNING)`。

**17.7 異構機隊只有一台在飛** → §15.2 第 7 條。快速檢查
`print({r.req_id: r.kind for r in coord.last_inputs.payload_reqs.values()})`（應同時有 lidar 與 rgb）與
`print(coord.id_map)`（應是 `{1:'A1', 2:'B2'}`）。

**17.8 `total_makespan` 比舊的大很多，是退步嗎** → 不是（§9.5）。要比較請比 `total_distances` 或 `total_length_m`。

**17.9 縮點後航點太少／太多** → `HvDccppCoordinator(..., reduce_waypoints=False)` 或
`from_mission_plan(..., reduce=False)` → 387–494 點／機（供地圖細節／除錯）。`wp_radius_m` 預設 `min(R_eff/2, 30)`
（規格 §8.5），調小 → 點更多更貼弧段。縮點**不影響**五個 `*_length_m`。

---

## 18. 後續工作與建議

**18.1 寫 ADR**（`REPO/docs/refactor_audit_2026.md:175` 明講架構級變更應寫 ADR，範本 `REPO/docs/adr/template.md`）：
① **以 hv_dccpp 取代 DCCPP 規劃核心**——Context = 既有 DCCPP 無異構／無能量／無風、`AltitudePlanner` 在 live 路徑未被呼叫、
`fov_width` 未被讀；Decision = 在 `plan_coverage_dccpp` 內加旗標分支、保持 6 鍵回傳不變；
Consequences = 下游零改動、makespan 語意改變、既有 golden 需另立、`vehicle_type` 限 `vtol`。
② **規劃移入背景執行緒**——Context = `main_window.py:4753-4776` 同步阻塞、大情境 43 s；
Decision = 新增 `ui/workers/planning_worker.py`；Consequences = 後處理改由 signal 觸發、取消只在階段邊界生效。
順帶一提，`refactor_audit_2026.md:167` 已規劃「拆 `plan_coverage_dccpp` → `core/dccpp/DccppPipeline`，方法保留為 facade」
——**hv_dccpp 正好佔這個位置**；若那次重構先發生，§5 的插入點改成新的 `DccppPipeline` 即可。

**18.2 UI 逐機設定**：目前最大的表達力缺口（`main_window.py:4722-4744` 連「這台是 A、那台是 B」都表達不了）。
建議一張 `QTableWidget`：drone_id｜構型｜感測器｜AUW(kg)｜可用(Wh)｜作業高度(m AGL)，直接餵
`class_of`／`energy`／`payload_reqs` 三個參數，一次補齊 §8.2 表列的大半欄位。

**18.3 把 `fov_width` 真正接起來**：短期——在 hv 分支下把 `HvUavState.fov_width`（轉接層已填該機主感測器於作業高度的
**實際刈幅**）顯示在機隊列表與地圖 tooltip（實測 160 m AGL 時 A 224 m vs B 235 m）。
長期——讓 `region_divider.decompose_by_fov` 接受逐機刈幅，但那等於重做既有 DCCPP 的區域分解，投報率不如直接用 hv。

**18.4 能量欄位進 UI**：表 2 明列「當架次 AUW」與「可用電量」是任務輸入，AeroPlan 完全沒有。
在 §18.2 的表格加兩欄，並在結果面板顯示逐機餘裕（`up.energy_used_wh` / `up.energy_margin_wh`；
實測 A1 310.8/589.2 Wh、B2 111.3/142.7 Wh）。**餘裕 < 0 應紅字警示**——這是既有系統完全沒有的安全資訊。

**18.5 其他**：把 `main_window.py:152 self.obstacles` 轉成 hv 的 `Obstacle` 放進 `MissionInputs`
（電塔／輸電線是 DEM 處理不了的，表 2 明列）；用 `export_all` 產生的 `<uav>_rally.txt` 填滿
`geofence_manager.py:467` 的空 stub；在 `SITLLink` 加 `upload_rally(points)`（`mission_type=2`）比照
`upload_fence_zones :268`；殘餘 4-D 衝突需規格 §13.6 步 (iii)；多起飛點支援（論文 Table 2 對不齊的主因）。

---

## 附錄 A：adapter 公開 API 速查表

模組 `hv_dccpp.integration.aeroplan_adapter`；頂層別名（延遲載入）`hv.HvDccppCoordinator` 等。

```python
HvDccppCoordinator(*, platform_a=PLATFORM_PIONEER, platform_b=PLATFORM_HERO,
                   class_of=None,                        # ★ 實務必填（§10.3）
                   sensor_a=None, sensor_b=None, payload_reqs=None,
                   cfg=None,                             # None → HvPlannerConfig(dem_step_m=25.0)
                   fence_zones=None, alternates=None, wind=None,     # (speed_mps, dir_from_deg)
                   c2_range_km=25.0, terrain_clearance_m=None,       # None → max(36, 3σ+30)
                   waiver_max_agl_m=200.0, dem_step_m=50.0,
                   dem_margin_m=1500.0,                  # ★ 請傳 3000.0（§11.1）
                   progress=None, on_report=None, energy=None, home=None,
                   types=None, reduce_waypoints=True, raise_on_no_go=True)

# 方法
.plan_coverage_dccpp(areas, drones, coverage_width_m, overlap_rate=0.1, auto_scan_angle=True,
                     coverage_altitude=100.0, terrain_func=None, enable_altitude=True,
                     mounting_angle_deg=0.0, dem_path="", coordination_mode="uncoordinated",
                     runway_bearing_deg=None, runway_length_m=None, landing_bearing_deg=None,
                     landing_rollout_m=0.0) -> dict          # 6 鍵
.upload_tuples(*, is_vtol=True, cruise_speed_mps=None, turn_radius_m=None,
               vtol_transition_alt_m=100.0) -> dict[int, list[tuple]]     # 8-tuple
.qgc_waypoints(*, key='drone_id') -> dict[str, list[str]]
.events_from_telemetry(frames, **kw) -> list[Event]
# 屬性（規劃後）：last_inputs / last_ctx / last_plan / last_result / notes / id_map

# 轉換函式
to_mission_inputs(areas, drones, *, coverage_width_m, overlap_rate=0.1, coverage_altitude=100.0,
                  terrain_func=None, dem_path='', home=None, fence_zones=None, class_of=None,
                  platform_a=PLATFORM_PIONEER, platform_b=PLATFORM_HERO, sensor_a=None, sensor_b=None,
                  payload_reqs=None, alternates=None, wind=(0.0, 0.0), c2_range_km=25.0,
                  terrain_clearance_m=None, waiver_max_agl_m=200.0, max_agl_m=120.0, dem_step_m=50.0,
                  dem_margin_m=1500.0, base_msl_m=0.0, weather_temp_c=25.0, mounting_angle_deg=0.0,
                  hfov_deg=0.0, vfov_deg=0.0, energy=None, warnings=None) -> (MissionInputs, id_map)
to_planner_config(params, *, base=None) -> HvPlannerConfig     # 只 min_separation_m / alt_offset_m / hv_cfg
from_mission_plan(plan, ctx, *, areas, drones, id_map=None, types=None, reduce=True,
                  wp_radius_m=None) -> dict                    # 6 鍵
to_upload_tuples(built_path, *, is_vtol=True, cruise_speed_mps=15.0, turn_radius_m=50.0,
                 vtol_transition_alt_m=100.0, default_alt_m=100.0, emit_change_speed=True) -> list[tuple]
fence_zones_to_nofly(zones, *, alt_ref='msl', home_msl_m=0.0, circle_segments=16,
                     include_categories=('NFZ', 'THREAT')) -> tuple[NoFlyZone, ...]
terrain_func_to_dem(terrain_func, bounds_latlon, step_m=50.0, *, base_msl_m=0.0, sigma_z_m=2.0,
                    is_dsm=False, max_nodes=4_000_000) -> DemGrid
telemetry_frames_to_events(frames, plan, ctx, cfg, *, id_map=None, uav_id_of=None, t0_epoch_s=None,
                           link_timeout_s=5.0, now_s=None, last_link_ok=None,
                           return_telemetry=False) -> list[Event] | (events, {uav_id: Telemetry})
plan_to_aeroplan_waypoints(plan, ctx, cfg=None, *, header=True, camera_trigger=True,
                           id_map=None, key='uav_id') -> dict[str, list[str]]
qgc_wpl_lines(items, *, header=True) -> list[str]
use_native_types(module_map, *, set_default=True) -> HvTypeRegistry
default_class_of(drone) -> 'A' | 'B'
```

**輸出型別**（duck typing，欄位名照抄 AeroPlan）：
`HvSegmentLabel`（5 值）／`HvVehicleType`（MULTIROTOR/FIXED_WING/VTOL）／
`HvBuiltWaypoint(lat, lon, alt, heading_compass_deg, segment_type)` + property `heading_deg` + `to_tuple()`／
`HvBuiltPath(uav_id, waypoints, entry/operation/transfer/takeoff/landing_length_m, hv_uav_id, cls, makespan_s)`
+ property `total_length_m` + `to_latlon()`/`to_latlon_list()`/`to_latlon_3d_list()`／
`HvUavState(uav_id, position, heading, turn_radius, speed, vehicle_type, fov_width)`／
`HvOperationSegment(index, left_point, right_point, waypoints, length_m, covered, assigned_drone_id)` + `entry_points()`／
`HvCoveragePath(area_id, area_polygon, scan_angle_deg, operations, priority)` + `get_uncovered_operations()`／
`HvMdtspResult(area_id, uav_assignments, path_matrix, decision_matrix, total_distances, makespan（秒）, assembled_paths)`／
`HvTypeRegistry`（frozen，8 個型別欄位；模組層級 `default_types`）／`AeroPlanAdapterError(RuntimeError)` 有 `.errors`。

**常數**：`PHASE_TO_SEGMENT`（14→5，§9.1）／`SEGMENT_NAMES`／`PARAM_KEY_NOTES`（UI 25 鍵，§8）／
`MAV_CMD = {'NAV_WAYPOINT':16,'NAV_LAND':21,'NAV_TAKEOFF':22,'NAV_VTOL_TAKEOFF':84,'NAV_VTOL_LAND':85,'DO_CHANGE_SPEED':178,'DO_VTOL_TRANSITION':3000}`／
`DEFAULT_ENERGY_A`／`DEFAULT_ENERGY_B`（§10.6 的兩組值）。

**常用 hv_dccpp 頂層 API**：`hv.build_context` / `hv.preflight` / `hv.plan_mission` /
`hv.build_validation_cache` / `hv.validate(..., t_sync_s=…)` / `hv.export_all(..., alt_frame="relative",
takeoff_delay="operator")` / `hv.cache_from_plan` / `hv.replan` / `hv.PLATFORM_PIONEER` / `hv.PLATFORM_HERO` /
`hv.PlannerConfig`（★ 請用別名 `HvPlannerConfig`）。

---

## 附錄 B：一頁式檢查清單

**安裝**
- [ ] `pip install -e "C:/Users/joy46/OneDrive/文件/vtol演篹法"`（在 AeroPlan 的 venv 內）
- [ ] `python -c "import hv_dccpp as hv; print(hv.__version__)"` → `0.1.0`；`pytest <hv>/tests/test_aeroplan_adapter.py -q` → `48 passed`
- [ ] `REPO/requirements.txt` 加 `-e file:///…`，並把 numpy/scipy/matplotlib 下限提高
- [ ] `REPO/aeroplan_studio.spec:143` 後加 `hiddenimports += collect_submodules('hv_dccpp')`
- [ ] 重跑 AeroPlan 自己的 `pytest`（確認相依升級沒打壞東西）

**接縫（方式 A）**
- [ ] `swarm_coordinator.py:395 __init__` 加 `use_hv_dccpp` 等 9 個屬性
- [ ] `swarm_coordinator.py:1076`（DEM 載入之後）插入 hv 分支，含 `try/except ImportError` 降級
- [ ] `main_window.py:4754` 設 `coordinator.use_hv_dccpp`（**限 `vehicle_type == 'vtol'`**）
- [ ] 提供 `hv_class_of`（**不提供的話整隊會被當成 Pioneer**）
- [ ] `dem_margin_m=3000.0`（不要用預設 1500）；`c2_range_km` 給實測值；`alternates` 由操作員在地圖上選
- [ ] 確認 `coverage_altitude ≥ 120`（否則被夾，記 `coverage_altitude_clamped`）

**執行緒（必做）**
- [ ] 新建 `ui/workers/__init__.py`（空）與 `ui/workers/planning_worker.py`（§7.2 全文）
- [ ] `HvPlanningWorker` 存成 `self._hv_worker`（**不能是區域變數**）
- [ ] `main_window.py:4844` 之後的後處理抽成 `_display_dccpp_result(result)`，由 `finished_ok` 觸發
- [ ] `progress`→狀態列；`failed`→`QMessageBox`；`report`→logger + 統計面板
- [ ] 視窗關閉時 `self._hv_worker.wait(3000)`；worker 內**不碰任何 widget**

**UI**
- [ ] 決定方式 A 或 B；方式 B 加 `hv_dccpp_requested` signal 與 `main_window.py:486` 旁 connect
- [ ] 至少新增：構型 A/B 台數、作業高度、豁免天花板、C2 半徑、風速風向
- [ ] hv 分支下**取消勾選** `collision_avoidance`；考慮略過 `_apply_fence_avoidance_to_dccpp_result`（`:4844`）

**匯出與上傳**
- [ ] `.waypoints`：`coord.qgc_waypoints(key='drone_id')`
- [ ] SITL：**用 `coord.upload_tuples()`，不要用 `_build_wps_for_path`**（`main_window.py:6115` 起飛遺漏 bug）
- [ ] `export_all(..., takeoff_delay=?)`：`"operator"` 就必須逐機延遲 `MISSION_START`；`"nav_delay"` 就**不可**再延遲
- [ ] 上傳後確認第一項是 `DO_CHANGE_SPEED(178)`、第二項是 `NAV_VTOL_TAKEOFF(84)`

**驗證**
- [ ] SITL：兩構型高度不同（分層）、水平間距全程 ≥ `d_sep_m`
- [ ] 無 SITL：`tools/mock_autopilot.py --dump received_mission.json` 檢查飛控實收任務項
- [ ] `on_report` 的 `status` 顯示給操作員；`uncovered` 非空要紅字警示
- [ ] 逐機能量餘裕 `up.energy_margin_wh` 顯示在結果面板
- [ ] 另建 `REPO/tests/test_hv_dccpp_adapter.py`，**不要改** `test_dccpp_pipeline.py` 的 golden

**文件**
- [ ] 寫兩份 ADR（`REPO/docs/adr/`）：換規劃器、規劃移入執行緒
- [ ] 在 `REPO/docs/PROJECT_STATUS.md` 記錄 hv_dccpp 的位置與旗標
- [ ] 把 §9 的慣例對照表貼進團隊 wiki（座標／航向／高度基準最容易出錯）

---

**文件結束。** 有疑問先查 §15 錯誤對照表與 §17 FAQ；API 細節以
`hv_dccpp/integration/aeroplan_adapter.py` 的 docstring 與 `tests/test_aeroplan_adapter.py` 的 48 項斷言為準
（那才是可執行的契約）。

---

## 附錄 C：2026-09-06 實機接入紀錄與勘誤

本附錄記錄**第一次真的把 hv_dccpp 接進 AeroPlan Studio** 的結果（有 PyQt6 的那台機器）。
本文 §1–§18 是在**沒有 PyQt6** 的機器上寫的，以下是實際接上後必須修正的部分。

### C.1 環境（實測）

| 項目 | 值 |
|---|---|
| AeroPlan 儲存庫 | `C:/Users/joy46/Documents/Projects/aeroplan-studio`，分支 `quad_swarm` |
| hv_dccpp 路徑 | `C:/Users/joy46/Documents/Projects/vtol演篹法`（**不是** 本文各處寫的 OneDrive 路徑） |
| Python | 3.14.3（系統安裝，**沒有 venv**）；hv 宣告 `>=3.11`，3.14 實測無問題 |
| Qt | PyQt6 6.11.0 / Qt 6.11.0 / PyQt6-WebEngine 6.11.0 |
| 既有套件 | numpy 2.3.5、scipy 1.17.0、shapely 2.1.2、matplotlib 3.10.8 ⇒ **不需安裝任何新第三方套件**（§2.1 成立） |
| 安裝指令 | `python -m pip install -e "C:/Users/joy46/Documents/Projects/vtol演篹法" --no-build-isolation` |
| `pytest tests/test_aeroplan_adapter.py` | **48 passed**（§2.5 的預期輸出完全吻合） |
| §3 快速上手腳本 | `total_makespan = 1479.4 s`、A1 74 航點 / B2 44 航點、`id_map={1:'A1', 2:'B2'}`、上傳 78 + 48 項 —— **與 §3「真實輸出」逐項相同**。只有 `timing_ms.total` 較快（595 ms vs 909 ms，機器差異） |

### C.2 行號勘誤

**`mission/swarm_coordinator.py` 的行號全部仍然正確**（`__init__` :395、DEM 區塊 :1064-1075、
插入點 :1076、`_R` :1184、v2 return :1291-1298）。

**`ui/main_window.py` 的行號全部失效**（該檔已長到 7360 行，位移 +460 ~ +860）：

| 本文寫的 | 實際（2026-09-06） |
|---|---|
| `:205` home_point | `:458` |
| `:486` signal connect | `:741` |
| `:4673` `_on_dccpp_coverage_requested` | `:5133`（整支 5133-5567） |
| `:4709` areas / `:4721-4744` drones | `:5173` / `:5192-5205` |
| `:4754` coordinator 建構 | `:5215` |
| `:4844` 起的後處理 | `:5246-5562` |
| `:5014` `self._dccpp_result` | `:5517` |
| `:5046-5054` `_dccpp_metrics` | `:5554-5562` |
| `:6041-6210` SITL 上傳 | `on_sitl_upload_mission` :6903-7176 |
| `:6045-6159` `_build_wps_for_path` | 巢狀 closure :6996-7115 |
| `:6115-6116` 起飛 bug | :7068-7069（**bug 仍在**，見 C.4） |
| `:6237` `on_sitl_launch` | `:7387`；spawn home 推導 :7504-7536 |
| `parameter_panel.py:2488` `get_dccpp_params` | `:2512-2546`（仍是 25 鍵） |
| `parameter_panel.py:1956` `create_dccpp_panel` | `:1980-2449` |

⚠ 教訓：接入前先自己 grep 一次符號名，不要照抄行號。

### C.3 實際採用的接法

**方式 A（§5）+ 執行緒（§7）+ 一個新的 UI 群組**，而不是方式 B：

* `mission/swarm_coordinator.py`：`__init__` 加 9 個 `hv_*` 旗標；`:1076` 插入 hv 分支；
  另外**補上 `vtol_transition_alt_m` 參數**（見 C.4 第 1 條）。
* `ui/workers/planning_worker.py`：`HvPlanningWorker(QThread)`，但**接受已設定好旗標的
  `SwarmCoordinator`**，而不是自己組 `HvDccppCoordinator` —— 這樣 GUI 與 headless 共用同一個接縫，
  避免「兩條啟動路徑參數覆蓋不一致」那類靜默失效。
* `ui/main_window.py`：後處理（5246-5562）原封不動切成 `_display_dccpp_result(...)`，
  由同步路徑或 worker 的 `finished_ok` 觸發。
* `ui/widgets/vtol_recon_panel.py`（**入口在這裡**）：「VTOL 偵察」分頁的「5. 規劃」加了一個
  **規劃演算法**下拉 —— `DCCPP（既有實作）` 與 `HV-DCCPP 異構雙構型（本專案）`。
  選到 HV 才展開「HV-DCCPP 參數」，內含兩個子群組：
    * **機隊組成** —— Pioneer PNP 與 HERO PNP **各自一格台數**，
      總台數 = 兩者相加（HV 模式下「4. 群飛編組」的 VTOL 數量改為唯讀的鏡子，
      同一個數字不會有兩個地方可改）。摘要標籤即時顯示兩型的巡航／轉彎／最低高度，
      只挑一種時提示「同構機隊」，空機隊按規劃會當場擋下。
    * **任務限制** —— 豁免 AGL 上限、C2 半徑、對地淨空、風速風向。
  機隊組成那兩列各配一張**機體照片**（`ui/resources/images/pioneer_pnp.png` /
  `hero_pnp.png`，該資料夾整包已被 PyInstaller 收進去）；照片不在時那一格顯示
  該放的檔名，面板照常運作。滑過任一格會出現含大圖的 tooltip。
  `get_params()` 多出 `planner` / `hv_dccpp` / `hv_num_a` / `hv_num_b` 與其餘 `hv_*` 鍵，
  按鈕字樣也跟著換。其餘欄位（作業高度、條帶間距、最小間距、高度錯開量）兩套演算法共用。
* **看得出哪一架是哪一型**：`from_mission_plan` 回傳的 `HvBuiltPath.cls` 會被帶進
  `swarm_data['drones'][i]['uav_class']` / `['platform']`，於是
    * 3D（Cesium）同構型用同一色系（Pioneer 暖色、HERO 冷色），色系內依序取深淺，
      線段標籤帶機型；同構規劃（`uav_class` 缺席）維持原本「1 架 = 1 色」，逐位元不變。
    * 偵察頁結果表多一欄「構型」，摘要行寫成 `2/5 架｜Pioneer PNP×1、HERO PNP×1｜…`，
      並在有飛機沒被分配到工作時明寫「N 台沒有分到工作」——
      ⚠ 規劃器**不會**告訴你這件事（實測 3 台的小作業區只會用到 2 台，
      `allocation` 就只有兩個 uav_id，沒有任何 note）。
* `ui/widgets/parameter_panel.py`（一般 DCCPP 分頁）：**刻意不放** HV 入口。
  同一個旗標有兩個入口只會讓人搞不清楚自己按的是哪一套。
* 決策記錄：AeroPlan 側 `docs/adr/0007-hv-dccpp-planner.md`、`0008-planning-in-worker-thread.md`。

⚠ 偵察頁按下規劃後**不能**再當場讀 `self._dccpp_result`：HV 走背景執行緒，那時結果還沒到。
`_on_dccpp_coverage_requested` 因此多了一個 `on_done(ok: bool)` 回呼，成功／失敗／取消三條路
都會叫到一次，摘要表由 `_on_vtol_recon_planned` 接手填。

### C.4 §5–§14 需要補充的四件事

1. **`plan_coverage_dccpp` 當時根本收不下 GUI 送的參數。**
   `ui/main_window.py`（未 commit 的工作樹）會傳 `vtol_transition_alt_m=`，
   而 `mission/swarm_coordinator.py:975` 沒有這個參數也沒有 `**kwargs`
   ⇒ **每一次按下 DCCPP 規劃都丟 TypeError**，被那個大 `except` 接住顯示「DCCPP 失敗」。
   接入時必須先補這個參數，否則連基準線都跑不起來。

2. **`ui/main_window.py` 當時完全 import 不了。**
   它從 `core.geometry.polygon` 匯入 `coverage_angle_to_compass` / `fw_scan_angle_to_compass`，
   而那兩支在任何 commit、stash 或 worktree 裡都不存在（同一批工作只有 UI 那半進了樹）。
   已依 `tests/test_fence_scan_angle.py` 的可執行契約與**真的規劃器**反解後補回：
   `coverage_angle_to_compass(a) = (90+a) % 360`、`fw_scan_angle_to_compass(a) = (-a) % 360`。

3. **§12.3 的起飛 bug 確認仍然存在**（`ui/main_window.py:7068`）：
   `if idx == 0: continue` 排在 `if not takeoff_emitted` **之前**，而 hv 縮點後的起飛段
   確實只有一個航點且就在索引 0（本次實測 A1/B1 皆是）⇒ 走 `_build_wps_for_path` 的話
   整份任務不會有起飛指令。已照 §12.3 在 `on_sitl_upload_mission` 內加分支：
   目前 `_dccpp_result` 來自 hv 時改用 `coord.upload_tuples()`。

4. **`QtWebEngine` 在自動化環境裡一 `setHtml` 就 abort**（offscreen 與真實平台都一樣，
   Windows 端 exit code `0xC0000409`）⇒ **建不出真的 `MainWindow`**，§14 的 GUI 端到端驗證
   只能由人在互動環境操作。測試改用本專案既有的「未綁定方法 + 假 self」寫法，
   拿**真的 hv 規劃結果**跑完整條後處理（`tests/test_hv_dccpp_adapter.py`）。

### C.4.5 對抗式審查抓到的**我們自己**的兩個 bug（都已修）

1. **關視窗會讓程式 abort**。worker 原本用 `parent=self` 建立 ⇒ 它是 MainWindow 的 QObject 子物件；
   規劃停不下來，`wait(3000)` 逾時後視窗被銷毀，Qt 對一個**還在跑的 QThread** 呼叫解構子 = qFatal。
   實測 8 機情境光是不可中斷的那一段就要 5.7 s，一定會逾時。
   已改成 `parent=None` + 關窗時把執行緒「寄放」到模組層級（拆掉所有信號），程式結束前再等它收尾。
2. **取消功能是假的**。原本用「從 progress 回呼拋例外」中止規劃，但
   `HvDccppCoordinator._tick`（`aeroplan_adapter.py:1705-1710`）把回呼包在 `try/except Exception: pass`
   ⇒ 例外根本傳不出去。已改成誠實版：`stop()` 只代表「結果不採用」（新增 `cancelled` 信號），
   並在 docstring 與 UI 訊息裡明說「規劃無法中途取消」。
   👉 給手冊 §7.5 的訂正：那一節寫「`cancel()` 會在下一次 progress 回呼時拋例外而中止」，**這是做不到的**。
3. **禁飛區在錯的執行緒被讀取**。`FenceZoneRegistry` 是主執行緒的 QObject 單例，
   原本由跑在背景執行緒的接縫自己去讀。已改成 GUI 端（`_configure_hv_dccpp`）先取好，
   放進 `coordinator.hv_fence_zones` 再交出去；接縫只在沒人給時才自己查（headless 呼叫端）。
   順帶把「取不到 fence zone」從 debug 提升為 warning —— 靜悄悄地把所有禁飛區丟掉是不能接受的。

### C.5 附錄 B 檢查清單的完成度

* **安裝**：全部完成（`requirements.txt` 改為註解說明 + 提高 numpy/scipy/matplotlib 下限；
  刻意不寫 `-e file:///…` 絕對路徑，換機會讓整份 requirements 安裝失敗，而 hv 是選用的）。
  `aeroplan_studio.spec` 已加 `collect_submodules('hv_dccpp')`（包在 try/except，未安裝時不影響打包）。
* **接縫（方式 A）**：全部完成。`dem_margin_m=3000.0` 已用值層級測試鎖住。
* **執行緒**：全部完成（worker 存成成員、後處理抽成方法、progress/failed/report 都接了、
  closeEvent 會等 3 秒）。**取消**只在階段邊界生效這件事寫在模組 docstring 裡。
* **UI**：完成基本欄位；**備降點仍缺地圖點選**（未給時系統自動放一點，那個點不見得能降落）。
* **匯出與上傳**：`.waypoints` 沿用既有流程；SITL 改走 `upload_tuples()`。
  `export_all` 的 `takeoff_delay` 尚未接 UI。
* **驗證**：`tests/test_hv_dccpp_adapter.py` **29 項全過**（含背景執行緒整條線、關窗寄放、
  面板 → 規劃器的參數跳躍）。回歸比對：同一組 103 個測試檔，改動前 462 failed / 3021 passed、
  改動後 457 failed / 3026 passed ⇒ **零新增失敗**。
  ⚠ 整套 `pytest tests` 不能一次跑完：`tests/test_vtol_prearm_real.py` 會啟動真的 SITL 並卡住。
  **真 SITL 尚未跑過 hv 產生的任務**（需人在互動環境啟動）。
* **文件**：ADR 0007/0008 已寫，`docs/PROJECT_STATUS.md` 已記錄。

### C.6 接手者要注意的三個 AeroPlan 側現況

1. **地圖的自動圍籬從來沒有建成功**：`ui/widgets/dual_map_widget.py:602` 呼叫
   `mgr.build(..., scan_angle_deg=)`，而 `mission/geofence_manager.py` 沒有這個參數
   ⇒ 每次都 TypeError 被 except 吞掉。與 hv 無關，但看畫面時不要誤以為是 hv 造成的。
2. **既有 DCCPP 對 VTOL 仍是跑道起降**：`DCCPPPathBuilder` 缺
   `prepend_vtol_takeoff` / `append_vtol_landing`（同樣是只進了一半的工作）。
   走 hv 分支時不受影響（hv 自己有垂直起降走廊）。
3. **該工作樹有約 85 個未 commit 的檔案、12 個既有測試收集錯誤**，整套 `pytest tests` 一次跑會在
   某個檔硬當。驗收請分批跑，或只跑相關子集。
