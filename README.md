# AeroPlan Studio — Collaborative UAV Mission Planning Suite

> 舊名：UAV Path Planner / DWA_path_planner

**版本**: 2.6.0
**授權**: MIT
**Python**: >= 3.10

## 專案概述

**AeroPlan Studio** 是一套面向多機協同作業的專業級無人機任務規劃平台，整合 2D/3D 互動地圖、覆蓋與點對點路徑規劃、DCCPP 多機協同最佳化、Dubins 三維軌跡生成、ArduPilot SITL 多機模擬與 Mission Planner 相容的 MAVLink 任務匯出於單一 PyQt6 桌面應用。支援多旋翼、固定翼與 VTOL 載具；所有 2D/3D 地圖資源（Leaflet / Cesium 1.115）皆可離線運作。

### 主要特性

- **2D / 3D 雙模式地圖**: 頂部按鈕一鍵切換 — 2D Folium/Leaflet（衛星 + 邊界編輯）與 3D Cesium（含高度視覺化、地形吸附、SITL UAV 即時追蹤）；地圖視角會自動記住，重新生成路徑時不再跳回預設座標
- **離線 / 線上自動切換**: `assets/cesium/` 與 `assets/leaflet/` 存在時自動使用本地資源，否則 fallback CDN，無網路也可正常開啟
- **內建 SITL 模擬器整合**: 一鍵啟動 `ArduPlane.exe` / `ArduCopter.exe` / `ArduPlane.exe -f quadplane`（VTOL），自動建立 MAVLink 連線並把 UAV 即時位置投到 3D 地圖上
- **VTOL QuadPlane SITL**: 完整支援垂直起降固定翼 — 動態生成 `vtol_default.parm`（~120 參數，參考 Alti Transition 真機）、`NAV_VTOL_TAKEOFF` 指令、多旋翼起飛 → 固定翼巡航 → 多旋翼降落完整任務序列
- **Mission Planner 風格 HUD**: 即時顯示飛行模式、武裝狀態、姿態（Roll/Pitch）、地速/空速、爬升率、航向、油門、電量、GPS Fix/衛星數
- **多種規劃演算法**: Grid / Spiral / Circular / A* / RRT / RRT* / Dijkstra / DWA
- **多飛行器支援**: 多旋翼（DJI Mavic 3、Phantom 4 Pro、Mini 3 Pro）、固定翼（衝浪者、Generic）、VTOL（QuadPlane）
- **固定翼三階段任務**: 起飛路徑 → 螺旋/同心圓掃描（轉彎半徑強制合規）→ 五邊進場降落（含 landing rollout 錨點偏移，整條五邊隨觸地點一併平移）
- **協同覆蓋任務（群飛）**: 多架無人機自動分工、加權成本智能分配、序列最佳化、按機分色視覺化
- **DCCPP 深度整合**: 基於論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning method for complex areas》（Defence Technology 47, 2025），整合 GreedyAllocator 多區域分配、IDP 路徑序列最佳化、Dubins 曲線連接、梯形 FOV 模型、DEM 地形雙線性插值、GDA 高度平滑、協調/非協調進入模式
- **DCCPP 多機防撞**: `core/dccpp/collision_avoidance.py` — 多架 UAV 路徑時空衝突檢測與迴避
- **🎯 蜂群末端打擊（Swarm Strike）**: N 目標 → 自動生成 N 架 UCAV，三段軌跡（起飛爬升 8° → Dubins 巡航 → 末端俯衝 θ_max）+ 高度錯層防撞；Cesium 視覺化含速度向量姿態、戰術色變、雷射鎖定、命中衝擊波；匯出每機獨立 QGC WPL 110 任務
- **障礙物避讓**: 碰撞檢測與智能避障
- **MAVLink 匯出**: QGC WPL 110 航點檔案匯出（Mission Planner / QGroundControl 相容）；群飛 / 蜂群打擊模式可逐機匯出獨立任務檔
- **即時路徑預覽**: 參數調整後自動重新生成路徑
- **座標系轉換**: WGS84 / UTM / 本地 ENU 座標互轉
- **地圖拖曳定圓**: 直接在地圖上拖曳定義螺旋/同心圓中心與半徑
- **多邊形編輯器**: 精確輸入或編輯角點座標
- **禁航區管理（NFZ）**: 多邊形/圓形禁航區視覺化與規劃迴避

## 快速開始

### 1. 安裝依賴

```bash
# 創建虛擬環境（推薦）
python -m venv venv
venv\Scripts\activate        # Windows
# source venv/bin/activate   # Linux/Mac

# 安裝依賴套件
pip install -r requirements.txt
```

核心依賴：

| 套件 | 用途 |
|------|------|
| `PyQt6` + `PyQt6-WebEngine` | GUI 與地圖顯示 |
| `folium` | 互動式地圖生成 |
| `numpy` / `scipy` | 數值計算 |
| `shapely` / `pyproj` | 幾何與座標投影 |
| `PyYAML` | 配置檔讀取 |
| `pymavlink` | MAVLink 航點匯出 + SITL 即時遙測接收 |
| `Cesium.js`（內建於 `assets/cesium/`） | 3D 地圖渲染（離線可用） |
| `Leaflet`（內建於 `assets/leaflet/`） | 2D 地圖渲染（離線可用） |

### 2. 啟動程式

```bash
# GUI 模式（預設）
python main.py

# 指定飛行器配置
python main.py --vehicle surfer

# 設定日誌等級
python main.py --log-level DEBUG

# 無介面模式（開發中）
python main.py --no-ui
```

### 3. 基本操作流程

1. **定義飛行區域** — 在地圖上左鍵點擊添加角點（至少 3 個），形成多邊形邊界
2. **選擇飛行器** — 右側面板選擇載具類型與型號
3. **選擇演算法** — 選擇路徑演算法（Grid、Spiral、Circular 等）
4. **調整參數** — 設定飛行高度、速度、航線間距、掃描角度等
5. **預覽路徑** — 點擊「預覽」按鈕或按 Enter 生成路徑
6. **匯出航點** — 匯出為 QGC WPL 110 (.waypoints) 或 CSV 格式
7. **切換 3D 視角** — 地圖頂部點 **🌐 3D Cesium**，可看到完整高度資訊（起飛爬升 / 任務 / 進場降落三段相連）

#### 2D / 3D 地圖切換

- 地圖頂部工具列有 **🗺 2D 衛星** 與 **🌐 3D Cesium** 兩顆按鈕
- 在任一模式下繪製的角點 / 路徑都會同步到另一模式
- 3D 模式支援：拖曳旋轉視角、滾輪縮放、🎯 飛向路徑、地形吸附、2D / 3D / 哥倫布三種投影
- 平移或縮放後的視角會被記住,下次重繪不會跳回預設位置

#### SITL 模擬連線（Mission Planner 等價，支援多機 + VTOL）

1. 從 Mission Planner 安裝目錄 `sitl/` 複製 `ArduPlane.exe`、`ArduCopter.exe` 與 `default_params/` 至專案 `sitl/` 資料夾（已於 `sitl/default_params/plane.parm`、`copter.parm`、`vtol_default.parm` 提供 SITL 友善預設）
2. 右側面板切換到 **🛰 SITL** 分頁
3. **單機**：選 `PLANE` / `COPTER` / `VTOL` → 點 **🚀 啟動** → 自動以 `tcp:127.0.0.1:5760` 連線
   - VTOL 以 `ArduPlane.exe -f quadplane` 啟動並套用動態生成的 `vtol_default.parm`（~120 參數，含 Q_ENABLE、Q_FRAME_CLASS、Q_A_RAT_*、Q_VELZ_MAX、Q_ASSIST_* 等）
4. **多機（DCCPP 模式）**：先在 **DCCPP** 分頁完成最佳化；再回到 SITL 分頁按 **🚀 啟動**，AeroPlan Studio 會：
   - 依 DCCPP 每台 UAV 的起飛點生成 N 個 SITL 實例（每個獨立 console 視窗）
   - 自動寫入 per-instance `identity.parm`（`SYSID_THISMAV`、`SIM_DRIFT_SPEED=0`、`SIM_RATE_HZ=400`）
   - 以 `tcp:127.0.0.1:5760 / 5770 / 5780 ...` 建立 N 條 MAVLink 連線
   - 每條連線 1 Hz 發送 GCS heartbeat 以避免 `FS_GCS → RTL`
5. 按 **上傳任務** → 依 uav_id 將每台 DCCPP 路徑分發至對應 SITL 實例（不同飛機上傳不同航線）
   - VTOL 路徑自動以 `NAV_VTOL_TAKEOFF (84)` 起飛指令取代 `NAV_TAKEOFF (22)`（使用 `vehicle_hint` 參數繞過 `MAV_TYPE=1` 模糊辨識）
   - VTOL 任務序列：垂直起飛 → 固定翼過渡 → 巡航 / 掃描 → 多旋翼過渡 → 垂直降落
6. HUD 多行 log（Mission Planner 風格）即時顯示模式 / ARMED / 姿態 / 速度 / 電量 / GPS / FC 訊息
7. **📂 參數檔** 按鈕：可一鍵載入外部 `.param` 檔批次寫入 SITL

> 也可手動連線：連線下拉選 `udpin:0.0.0.0:14550` / `tcp:127.0.0.1:5760` / `COM7,57600` 等任意 pymavlink 連線字串。

#### DEM 3D 真實地形顯示

1. 以 `sensors/dem_terrain.py` 載入 GeoTIFF 或 `.npy` 高程資料
2. Cesium 地圖會自動以 `CustomHeightmapTerrainProvider` + 雙線性插值重建地形表面
3. 開啟 3D 光照，起伏一目了然，任務路徑會沿真實高度貼合

#### 群飛協同覆蓋流程

1. 定義飛行區域（步驟同上）
2. 捲動右側面板至「**協同覆蓋設定**」區段
3. 設定無人機數量（2–8 台）、FOV 覆蓋寬度、重疊率
4. 點擊「**生成協同覆蓋路徑**」— 地圖以分色顯示各機路徑，右下角顯示圖例與統計
5. 點擊「**匯出群飛任務**」— 選擇目錄，自動為每架無人機產生獨立 `.waypoints` 檔案

#### 🎯 蜂群末端打擊流程（Swarm Strike）

1. 切換右側面板至 **🎯 蜂群打擊** 分頁
2. 點擊 **標記目標** → 在 3D Cesium 地圖上點擊 N 個地面目標（自動編號 TGT-1 ~ TGT-N）
3. 設定參數：巡航高度 / 巡航速度 / 最大俯衝角 θ_max（通常 30°~60°）/ 俯衝啟動距離 / 高度錯層間距
4. 點擊 **EXECUTE SWARM STRIKE** → 系統會：
   - 自動生成 N 架 UCAV（各機起飛點錯開，巡航高度依 `altitude_step_m` 錯層防撞）
   - 每機規劃三段軌跡：**起飛爬升段**（8° 爬升至巡航高度，黃色）→ **Dubins 巡航段**（最小轉彎半徑合規，綠色）→ **末端俯衝段**（依 θ_max 急降至目標，血紅色加粗）
   - 俯衝段渲染含：速度向量姿態、戰術顏色漸變、雷射目標鎖定、命中衝擊波
5. 點擊 **💾 匯出打擊任務 (QGC WPL)** → 為每架 UCAV 產出獨立 `.waypoints`（含 `DO_SET_HOME` / `DO_CHANGE_SPEED` / `NAV_TAKEOFF` / 巡航 / 俯衝 `NAV_WAYPOINT`）+ `SWARM_STRIKE_briefing.txt` 任務簡報

## 路徑規劃演算法

### 覆蓋任務（區域掃描）

| 演算法 | 說明 | 適用場景 |
|--------|------|----------|
| **Grid** | 網格掃描（之字形） | 農業噴灑、航拍測繪 |
| **Spiral** | 阿基米德螺線掃描（支援曲率係數、高度漸變） | 從外圍向中心搜索 |
| **Circular** | 同心圓擴張掃描（自適應密度過渡弧） | 環繞目標、逐圈上升拍攝 |
| **協同覆蓋（群飛）** | 多機分工並行掃描（FOV 分解 + 加權成本分配 + 序列最佳化） | 大面積快速掃測 |
| **DCCPP 最佳化** | 論文級完整管線：GreedyAllocator → IDP 排序 → Dubins 曲線 → GDA 3D 高度平滑 + 多機時空防撞 | 多固定翼 UAV 複雜區域 3D 覆蓋 |
| **蜂群末端打擊** | 三段軌跡：起飛爬升 → Dubins 巡航 → 末端俯衝；N 目標自動分配 N 架 UCAV + 高度錯層防撞 | 協同對地目標精準打擊 |

### 點對點路徑

| 演算法 | 說明 | 適用場景 |
|--------|------|----------|
| **A*** | 啟發式最短路徑搜索 | 已知環境中的最優路徑 |
| **RRT** | 快速探索隨機樹 | 複雜障礙物環境 |
| **RRT*** | RRT 最優化版本 | 需要路徑品質保證 |
| **Dijkstra** | 保證最短路徑 | 簡單環境最短路徑 |
| **DWA** | 動態窗口法 | 即時避障（需配合飛控） |

### 固定翼任務規劃

固定翼模式採用三階段任務規劃，確保與 ArduPlane 飛控相容：

| 階段 | 說明 |
|------|------|
| **Phase 1 — 起飛** | MAVLink TAKEOFF 指令 + 爬升轉場至掃描高度 |
| **Phase 2 — 掃描** | 螺旋掃描 / 同心圓掃描（含轉彎半徑強制合規檢查） |
| **Phase 3 — 降落** | 五邊進場（Crosswind → Downwind → Base → Final）|

**轉彎半徑合規性檢查**：根據機體最大傾斜角計算所需傾斜角，回傳 `ok / warning / critical` 三段狀態。

## 飛行器配置

### 多旋翼

| 載具 | 最大速度 | 飛行時間 | 抗風能力 |
|------|----------|----------|----------|
| DJI Mavic 3 | 19 m/s | 46 min | 12 m/s |
| DJI Phantom 4 Pro | 20 m/s | 30 min | 10 m/s |
| DJI Mini 3 Pro | 16 m/s | 34 min | 10.7 m/s |
| Generic Quadcopter | 15 m/s | 25 min | 10 m/s |

### 固定翼

| 載具 | 巡航速度 | 飛行時間 | 最小轉彎半徑 |
|------|----------|----------|--------------|
| 衝浪者 (Surfer) | 18 m/s | 90 min | 30 m |
| Generic Fixed Wing | 18 m/s | 120 min | 50 m |

### VTOL

| 載具 | 多旋翼巡航 | 固定翼巡航 | 飛行時間 |
|------|-----------|-----------|---------|
| Generic VTOL (QuadPlane) | 5 m/s | 20 m/s | 90 min（固定翼模式）|
| Alti Transition (參考真機) | 5 m/s | 22 m/s | 120 min（固定翼模式）|

> VTOL SITL 使用 `ArduPlane.exe -f quadplane` 啟動，預設參數檔 `sitl/default_params/vtol_default.parm` 內含 Q_ENABLE=1、Q_FRAME_CLASS、Q_FRAME_TYPE、Q_A_RAT_RLL/PIT/YAW_P、Q_VELZ_MAX、Q_ASSIST_SPEED / ALT 等 ~120 項多旋翼控制參數。

## 專案結構

```
aeroplan-studio/
├── main.py                          # 程式進入點（GUI / CLI 模式）
├── requirements.txt                 # 依賴套件
│
├── config/                          # 配置模組
│   ├── settings.py                  # 全局配置（地圖、安全、UI 參數）
│   └── vehicle_profiles.yaml        # 飛行器參數檔（多旋翼/固定翼/VTOL）
│
├── core/                            # 核心演算法
│   ├── base/                        # 基礎抽象類別
│   │   ├── vehicle_base.py          # 飛行器基類
│   │   ├── planner_base.py          # 規劃器基類
│   │   └── constraint_base.py       # 約束基類
│   │
│   ├── geometry/                    # 幾何與座標轉換
│   │   ├── coordinate.py            # WGS84/UTM/ENU 座標轉換
│   │   ├── transform.py             # 2D 仿射變換、旋轉座標系
│   │   ├── polygon.py               # 多邊形運算
│   │   ├── intersection.py          # 交點計算
│   │   └── region_divider.py        # 飛行區域分割
│   │
│   ├── global_planner/              # 全域路徑規劃
│   │   ├── coverage_planner.py      # 覆蓋規劃（Grid / Spiral / Circular）
│   │   ├── fixed_wing_planner.py    # 固定翼三階段任務規劃
│   │   ├── mdtsp_solver.py          # DCCPP 求解器（GreedyAllocator / IDPSolver / AltitudePlanner / CoordinatedPlanner）
│   │   ├── astar.py                 # A* 路徑規劃
│   │   ├── rrt.py                   # RRT / RRT* 路徑規劃
│   │   ├── dijkstra.py              # Dijkstra 路徑規劃
│   │   └── grid_generator.py        # 柵格地圖生成
│   │
│   ├── collision/                   # 碰撞檢測
│   │   ├── collision_checker.py     # 碰撞檢測器
│   │   ├── avoidance.py             # 避障策略
│   │   └── obstacle_manager.py      # 障礙物管理
│   │
│   ├── dccpp/                       # DCCPP 論文管線元件
│   │   ├── uav_models.py            # UAV / Area / Task 資料結構
│   │   ├── path_cost_calculator.py  # 加權成本函數
│   │   ├── task_allocator.py        # GreedyAllocator（Algorithm 1）
│   │   ├── idp_solver.py            # 改進動態規劃（Algorithm 2）
│   │   ├── altitude_planner.py      # GDA 高度平滑（Algorithm 3）
│   │   ├── area_processor.py        # 區域前處理
│   │   ├── dccpp_path_builder.py    # 完整 DCCPP 管線組裝器
│   │   └── collision_avoidance.py   # 多機時空衝突檢測 / 迴避
│   │
│   ├── strike/                      # 蜂群末端打擊規劃
│   │   └── terminal_strike_planner.py  # 三段軌跡（起飛→Dubins 巡航→末端俯衝）+ 高度錯層
│   │
│   ├── local_planner/               # 局域規劃
│   │   ├── dwa.py                   # 動態窗口法
│   │   ├── apf.py                   # 人工勢場法
│   │   └── mpc.py                   # 模型預測控制
│   │
│   ├── trajectory/                  # 軌跡優化
│   │   ├── smoother.py              # 路徑平滑
│   │   ├── spline.py                # 樣條插值
│   │   ├── time_optimal.py          # 時間最優軌跡
│   │   ├── dubins_trajectory.py     # Dubins 曲線軌跡生成（6 種路徑類型：CSC + CCC）
│   │   ├── dccpp_path_assembler.py  # DCCPP 航點組裝（IDP 結果 → Dubins 連接 → 完整可飛行路徑）
│   │   └── fixed_wing_constraints.py # 固定翼運動約束
│   │
│   └── vehicles/                    # 飛行器模型
│       ├── multirotor.py            # 多旋翼模型
│       └── fixed_wing.py            # 固定翼模型
│
├── sitl/                            # SITL 模擬器（從 Mission Planner 複製，git 不追蹤）
│   ├── ArduPlane.exe                # 固定翼 / VTOL SITL binary（VTOL 以 -f quadplane 啟動）
│   ├── ArduCopter.exe               # 多旋翼 SITL binary
│   ├── default_params/              # 預設參數檔（plane.parm / copter.parm / vtol_default.parm）
│   ├── plane/                       # 固定翼 SITL 工作目錄（eeprom + terrain）
│   └── plane/i3/                    # VTOL SITL 工作目錄（QuadPlane 實例）
│
├── assets/                          # Web 資源（離線使用，git 不追蹤）
│   ├── cesium/Build/Cesium/         # Cesium 1.115 完整 build（~20MB）
│   └── leaflet/                     # leaflet.js + leaflet.css
│
├── mission/                         # 任務管理
│   ├── mission_manager.py           # 任務管理器
│   ├── sitl_link.py                 # SITLLink (QThread) — pymavlink 接收 + TelemetryFrame
│   ├── sitl_launcher.py             # SITLLauncher — subprocess 啟動 ArduPilot SITL binary
│   ├── waypoint.py                  # 航點定義（MAVLink Waypoint / WaypointSequence）
│   ├── survey_mission.py            # Survey 測繪任務
│   ├── coverage_path.py             # 結構化覆蓋路徑（OperationSegment / CoveragePath）
│   ├── mavlink_exporter.py          # MAVLink / QGC WPL 110 匯出（QGC / JSON / KML / GPX）
│   ├── swarm_coordinator.py         # 群飛協調（SwarmCoordinator / SwarmMission）+ landing_rollout 傳遞
│   └── vtol_mission_exporter.py     # VTOL 專用匯出（NAV_VTOL_TAKEOFF / 過渡序列 / 垂直降落）
│
├── sensors/                         # 感測器模組
│   ├── camera_model.py              # 相機模型（GSD / 覆蓋率計算）
│   ├── fov_model.py                 # 梯形 FOV 地面投影模型（論文 Eq. 2-3，掛載角 → 梯形投影）
│   ├── dem_terrain.py               # DEM 地形雙線性插值（論文 Eq. 4，GeoTIFF / numpy）
│   ├── sensor_fusion.py             # 感測器融合
│   └── terrain_manager.py           # 地形管理基類
│
├── ui/                              # PyQt6 GUI
│   ├── main_window.py               # 主視窗（路徑生成協調 + SITL 整合）
│   ├── widgets/
│   │   ├── map_widget.py            # 2D 地圖（Folium + WebEngine + JS 事件 + 視角同步）
│   │   ├── cesium_map_widget.py     # 3D 地圖（Cesium.js + 高度視覺化 + UAV 即時追蹤）
│   │   ├── dual_map_widget.py       # 2D/3D 雙模式容器（QStackedWidget + 信號轉發）
│   │   ├── sitl_hud.py              # Mission Planner 風格 SITL HUD 面板
│   │   ├── parameter_panel.py       # 參數面板（基本演算法 / 協同覆蓋 / DCCPP / SITL 4 分頁）
│   │   ├── mission_panel.py         # 任務面板
│   │   └── polygon_editor.py        # 多邊形編輯器
│   ├── dialogs/
│   │   ├── camera_config.py         # 相機配置
│   │   ├── vehicle_config.py        # 載具配置
│   │   ├── export_dialog.py         # 匯出對話框
│   │   ├── obstacle_manager.py      # 障礙物管理
│   │   └── nfz_manager_dialog.py    # 禁航區管理
│   └── resources/
│       └── styles/                  # QSS 主題樣式（modern / light）
│
├── utils/                           # 工具模組
│   ├── logger.py                    # 日誌系統
│   ├── math_utils.py                # 數學工具
│   └── file_io.py                   # 檔案讀寫（YAML、航點匯出）
│
└── data/
    └── logs/                        # 執行日誌（依日期命名）
```

## 配置說明

### 系統配置 (settings.py)

| 配置類別 | 說明 |
|----------|------|
| `PathSettings` | 路徑與目錄配置 |
| `MapSettings` | 地圖預設位置與縮放 |
| `SafetySettings` | 飛行高度/速度/間距限制 |
| `ExportSettings` | 匯出格式設定 |
| `UISettings` | 視窗大小與標題 |

### 螺旋掃描進階參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `spiral_curvature` | 曲率係數（控制每圈緊密程度） | 1.0 |
| `spiral_alt_step` | 每圈高度增量 (m) | 0.0 |
| `spiral_base_altitude` | 螺旋基準高度 (m) | 同任務高度 |
| `circle_max_radius` | 最大半徑 (m) | 200.0 |
| `circle_min_radius` | 最小半徑 (m) | 0.0 |

### 協同覆蓋（群飛）參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `num_drones` | 無人機數量 | 3 |
| `coverage_width_m` | 每架無人機的 FOV 覆蓋寬度 (m) | 100.0 |
| `overlap_rate` | 相鄰條帶重疊率（0.0–0.5） | 0.1 |
| `auto_scan_angle` | 是否自動計算最佳掃描方向 | True |

**分配演算法**：加權成本函數 `Jk = w1×距離 + w2×路徑數 + w3×總長度 + w4×優先級`，以最大餘數法決定各區域分配機數，再以貪婪法指定最近的無人機。

**序列最佳化**：各機作業路徑以最近鄰啟發式排序，減少轉移段總距離。

### DCCPP 最佳化覆蓋參數

基於論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning method for complex areas》（Defence Technology 47, 2025）的完整管線整合。

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `vehicle_type` | 載具類型（multirotor / fixed_wing） | multirotor |
| `num_drones` | 無人機數量 | 3 |
| `fov_width_m` | FOV 覆蓋寬度 (m) | 100.0 |
| `overlap_rate` | 相鄰條帶重疊率（0.0–0.5） | 0.1 |
| `altitude` | 覆蓋高度 (m) | 100.0 |
| `turn_radius` | 最小轉彎半徑 (m)（固定翼專用） | 50.0 |
| `mounting_angle_deg` | 感測器掛載角 α_m（0°=垂直朝下，>0° 啟用梯形 FOV） | 0.0 |
| `dem_path` | DEM 地形檔案路徑（GeoTIFF / .npy，空=平地假設） | （無） |
| `coordination_mode` | 協調模式（uncoordinated / coordinated） | uncoordinated |
| `enable_altitude` | 啟用 GDA 高度平滑 | True |

**DCCPP 管線流程**：
1. **GreedyAllocator** (Algorithm 1, Eq. 30-31)：多區域 → 多 UAV 加權成本分配
2. **IDPSolver** (Algorithm 2, Eq. 32-34)：改進動態規劃最佳化作業路徑序列（MDTSP）
3. **DCCPPPathAssembler**：IDP 排序結果 → Dubins 曲線連接 → 完整航點路徑（Entry / Operation / Transfer 三段標記）
4. **AltitudePlanner** (Algorithm 3, Eq. 35-39)：2D 路徑 → 3D 高度剖面（含 GDA 梯度平滑）
5. **CoordinatedPlanner**（可選）：同步多 UAV 進入時間，較快 UAV 插入 S-turn 等待盤旋

### 固定翼進場參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `fw_pattern_alt` | 五邊飛行高度 (m) | 80.0 |
| `fw_downwind_offset` | 下風邊側偏距離 (m) | 200.0 |
| `fw_pattern_leg` | 標準腿長 (m) | 300.0 |
| `fw_final_dist` | 最終進場距離 (m) | 400.0 |
| `fw_takeoff_bearing` | 起飛方向 (°，0=北) | 0.0 |
| `fw_landing_bearing` | 降落進場方向 (°) | 180.0 |
| `landing_rollout_m` | 著陸滾行距離 (m)，整條五邊錨定於觸地點偏移 | 0.0（範圍 0–9999）|

### 蜂群末端打擊（Swarm Strike）參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `max_dive_angle_deg` | 最大俯衝角 θ_max (°) | 45.0 |
| `dive_initiation_dist_m` | 俯衝啟動距離（距目標水平距離, m） | 800.0 |
| `cruise_alt_m` | 基準巡航高度 (m) | 根據參數面板 |
| `cruise_speed_mps` | 巡航空速 (m/s) | 60.0 |
| `altitude_step_m` | 高度錯層間距 (m)，各機遞增此值防撞 | 30.0 |
| `cruise_step_m` | 巡航段航點間距 (m) | 50.0 |
| `dive_step_m` | 俯衝段航點間距 (m) | 20.0 |
| `takeoff_alt_m` | 起飛點地面海拔 (m) | 0.0 |
| `climb_angle_deg` | 起飛爬升角 γ (°) | 8.0 |
| `min_turn_radius_m` | 固定翼最小轉彎半徑 R_min (m) | `V²/(g·tan30°)` 動態計算 |
| `use_dubins_cruise` | 巡航段是否套用 Dubins 曲線 | True |

**匯出 MAVLink 指令對應**：`DO_SET_HOME (179)` → `DO_CHANGE_SPEED (178)` → `NAV_TAKEOFF (22)` → 巡航 `NAV_WAYPOINT (16, param2=R_min)` → 俯衝 `NAV_WAYPOINT (16, param2=0.25·R_min)`

## 快捷鍵

| 按鍵 | 功能 |
|------|------|
| Enter / Space | 生成路徑 |
| Escape | 清除路徑 |
| Delete / Backspace | 刪除最後一個角點 |
| Ctrl+N | 新建任務 |
| Ctrl+O | 開啟任務 |
| Ctrl+S | 儲存任務 |
| Ctrl+E | 匯出航點 |
| Ctrl+R | 清除全部 |
| Ctrl+M | 開啟多邊形編輯器 |

## 技術棧

| 類別 | 技術 |
|------|------|
| 語言 | Python 3.10+ |
| GUI | PyQt6 |
| 2D 地圖 | Folium + Leaflet + PyQt6-WebEngine |
| 3D 地圖 | Cesium.js 1.115 + PyQt6-WebEngine |
| 地圖互動 | JS click/drag/moveend → `pyqt://` URL scheme → Python signal |
| SITL 模擬 | ArduPilot SITL binary + pymavlink (subprocess + QThread) |
| 幾何 | NumPy, SciPy, Shapely |
| 座標投影 | pyproj + 自製 WGS84/UTM/ENU 轉換器 |
| 通訊 | pymavlink（QGC WPL 110 匯出 + 即時遙測接收） |
| 配置 | PyYAML |

## 版本紀錄

### v2.6.0 (Apr 2026)
- 🎯 **蜂群末端打擊規劃 (Swarm Strike)**：新增 `core/strike/terminal_strike_planner.py`，N 目標自動分配 N 架 UCAV
  - 三段軌跡：起飛爬升段（8° 爬升，黃色）→ Dubins 巡航段（最小轉彎半徑合規，綠色）→ 末端俯衝段（θ_max 急降，血紅色）
  - 多機高度錯層（`altitude_step_m` 遞增）防止空中相撞
  - Cesium 靜態路徑渲染：速度向量姿態、戰術顏色漸變、雷射目標鎖定、命中衝擊波
  - 蜂群打擊 QGC WPL 110 匯出：每機獨立 `.waypoints` + `SWARM_STRIKE_briefing.txt` 任務簡報
- ✨ **VTOL QuadPlane SITL 支援**
  - 動態生成 `sitl/default_params/vtol_default.parm`（~120 參數，參考 Alti Transition 真機）
  - 修正 VTOL 無法起飛：改用 `NAV_VTOL_TAKEOFF (84)` 取代 `NAV_TAKEOFF (22)`
  - 新增 `vehicle_hint` 參數繞過 `MAV_TYPE=1` 模糊辨識
  - VTOL 任務序列：垂直起飛 → 固定翼過渡 → 巡航 / 掃描 → 多旋翼過渡 → 垂直降落
  - 新增 `mission/vtol_mission_exporter.py` VTOL 專用匯出器
- 🐛 **修正 DCCPP 降落點與起飛點重合**：將 `landing_rollout_m` 串接 `SwarmCoordinator → DCCPPPathBuilder`，整個五邊進場 pattern 錨定於觸地點，隨 rollout 偏移整體平移
- ✨ **Landing rollout 範圍擴大**：500m → 9999m
- ✨ **DCCPP 多機防撞模組**：新增 `core/dccpp/collision_avoidance.py`，時空衝突檢測與路徑迴避

### v2.5.0 (Apr 2026)
- 🎉 **專案更名** `UAV Path Planner` → **AeroPlan Studio — Collaborative UAV Mission Planning Suite**
- ✨ **SITL 多機實例**：依 DCCPP 結果自動生成 N 架飛機，各自獨立 console、獨立 `SYSID_THISMAV`、獨立起飛點
- ✨ **DCCPP 多機任務分派**：按 uav_id 將不同路徑分別上傳至對應 SITL 連線
- ✨ **Mission Planner 風格多行 HUD log**（`QPlainTextEdit` 時戳彩色輸出）
- ✨ **外部 `.param` 檔載入按鈕** — 可批次套用到 SITL
- ✨ **DEM 真實 3D 地形** — `CustomHeightmapTerrainProvider` + 雙線性插值 + Cesium 光照
- ✨ **GCS 1Hz Heartbeat** 防止 AUTO → RTL 誤切換
- 🐛 修正 `--model +` 誤用為固定翼物理模型導致起飛翻滾
- 🐛 修正 `SIM_DRIFT_SPEED` 未歸零導致地面滑行
- 🐛 修正 DCCPP 多機皆上傳相同任務

### v2.4.0 (Apr 2026)
- ✨ **2D / 3D 雙模式地圖**：DualMapWidget 整合 Folium 與 Cesium，drop-in 替代原 MapWidget
- ✨ **3D Cesium 地圖**：完整高度視覺化、固定翼三段路徑相連、UAV 即時追蹤、地形吸附
- ✨ **內建 SITL 啟動器**：一鍵啟動 ArduPlane / ArduCopter binary 並自動連線
- ✨ **Mission Planner 風格 HUD**：模式 / ARMED / 姿態 / 速度 / 電量 / GPS 即時顯示
- ✨ **離線資源支援**：Cesium 1.115 與 Leaflet 1.9.4 內建於 `assets/`，無網路也可運作
- ✨ **2D 視角持久化**：地圖重繪不再跳回預設座標，記住使用者目前的 lat/lon/zoom
- 🐛 修正 3D 地圖固定翼三階段路徑斷層（takeoff→mission→landing 銜接）

### v2.3.0
- DCCPP 完整管線（GreedyAllocator + IDP + AltitudePlanner + Dubins + 協調模式）
- Dubins 曲線軌跡模組（CSC + CCC 6 種路徑類型）
- 群飛協同覆蓋與序列最佳化

## 致謝

- [Mission Planner](https://ardupilot.org/planner/) — 演算法參考
- [ArduPilot](https://ardupilot.org/) — MAVLink 協議
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) — 路徑規劃演算法參考
- [QGroundControl](http://qgroundcontrol.com/) — 航點格式參考
- [Folium](https://python-visualization.github.io/folium/) — 互動式地圖

### 論文參考

- **DCCPP 核心演算法**: Li et al., "Multiple fixed-wing UAVs collaborative coverage 3D path planning method for complex areas," *Defence Technology*, vol. 47, pp. 118-133, 2025. — GreedyAllocator、IDP、AltitudePlanner、梯形 FOV 模型、DEM 雙線性插值、協調覆蓋模式
