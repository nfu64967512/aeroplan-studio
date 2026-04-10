# AeroPlan Studio — Collaborative UAV Mission Planning Suite V2.5

## 專案概述

基於 Mission Planner 演算法核心，採用現代化 Python 架構設計的專業級無人機路徑規劃系統。
支援多旋翼、固定翼與 VTOL 載具，整合多種全域規劃演算法與論文級 DCCPP 協同覆蓋管線，提供互動式地圖操作介面。

## 設計原則

1. **策略模式 (Strategy Pattern)**: 飛行器類型可插拔切換
2. **工廠模式 (Factory Pattern)**: 路徑規劃器動態創建
3. **觀察者模式 (Observer Pattern)**: UI 與核心演算法解耦
4. **分層架構**: 全域規劃 → 局域規劃 → 軌跡優化
5. **向下兼容**: 所有新增功能均有預設值，不影響既有行為

## 專案結構

```
aeroplan-studio/
├── main.py                          # 程式進入點（GUI / CLI 模式）
├── requirements.txt                 # 依賴套件
├── README.md                        # 專案說明文件
├── PROJECT_STRUCTURE.md             # 本文件
│
├── config/                          # 配置模組
│   ├── settings.py                  # 全局配置（地圖、安全、UI 參數）
│   └── vehicle_profiles.yaml        # 飛行器參數檔（多旋翼/固定翼/VTOL）
│
├── core/                            # 核心演算法層
│   ├── base/                        # 基礎抽象類別
│   │   ├── vehicle_base.py          # 飛行器基類
│   │   ├── planner_base.py          # 規劃器基類
│   │   └── constraint_base.py       # 約束基類
│   │
│   ├── vehicles/                    # 飛行器模型
│   │   ├── multirotor.py            # 多旋翼模型
│   │   └── fixed_wing.py            # 固定翼模型
│   │
│   ├── geometry/                    # 幾何與座標轉換
│   │   ├── coordinate.py            # WGS84/UTM/ENU 座標轉換（CoordinateTransformer）
│   │   ├── transform.py             # 2D 仿射變換、旋轉座標系
│   │   ├── polygon.py               # 多邊形運算（PolygonUtils）
│   │   ├── intersection.py          # 交點計算
│   │   └── region_divider.py        # 飛行區域 FOV 分割（RegionDivider）
│   │
│   ├── global_planner/              # 全域路徑規劃
│   │   ├── coverage_planner.py      # 覆蓋規劃（Grid / Spiral / Circular）
│   │   ├── fixed_wing_planner.py    # 固定翼三階段任務規劃（起飛/掃描/降落）
│   │   ├── mdtsp_solver.py          # ★ DCCPP 求解器（論文核心）
│   │   │                            #   - GreedyAllocator: 多區域 UAV 分配（Algorithm 1, Eq. 30-31）
│   │   │                            #   - IDPSolver: 改進動態規劃路徑排序（Algorithm 2, Eq. 32-34）
│   │   │                            #   - AltitudePlanner: 2D→3D 高度規劃 + GDA 平滑（Algorithm 3, Eq. 35-39）
│   │   │                            #   - DCCPPSolver: 完整管線協調器
│   │   │                            #   - CoordinatedPlanner: 多 UAV 同步進入 + S-turn 等待
│   │   ├── astar.py                 # A* 路徑規劃
│   │   ├── rrt.py                   # RRT / RRT* 路徑規劃
│   │   ├── dijkstra.py              # Dijkstra 路徑規劃
│   │   └── grid_generator.py        # 柵格地圖生成
│   │
│   ├── collision/                   # 碰撞檢測與避障
│   │   ├── collision_checker.py     # 碰撞檢測器
│   │   ├── avoidance.py             # 避障策略
│   │   └── obstacle_manager.py      # 障礙物管理
│   │
│   ├── local_planner/               # 局域路徑規劃
│   │   ├── dwa.py                   # 動態窗口法（Dynamic Window Approach）
│   │   ├── apf.py                   # 人工勢場法
│   │   └── mpc.py                   # 模型預測控制
│   │
│   └── trajectory/                  # 軌跡優化與生成
│       ├── smoother.py              # 路徑平滑
│       ├── spline.py                # 樣條插值
│       ├── time_optimal.py          # 時間最優軌跡
│       ├── dubins_trajectory.py     # ★ Dubins 曲線軌跡生成
│       │                            #   - 6 種路徑類型：CSC (RSR/LSL/RSL/LSR) + CCC (RLR/LRL)
│       │                            #   - Pose3D (NED: x=North, y=East)
│       │                            #   - DubinsTrajectoryGenerator.calculate_path() / generate_waypoints()
│       ├── dccpp_path_assembler.py  # ★ DCCPP 航點組裝器
│       │                            #   - IDP 排序結果 → Dubins 曲線連接 → 完整可飛行航點路徑
│       │                            #   - SegmentLabel: ENTRY / OPERATION / TRANSFER 三段標記
│       │                            #   - AssembledWaypoint / AssembledPath 資料結構
│       │                            #   - 自動處理 ENU↔NED 座標轉換
│       │                            #   - 多旋翼跳過 Dubins 使用直線連接
│       └── fixed_wing_constraints.py # 固定翼運動約束（爬升角、傾斜角、轉彎半徑）
│
├── mission/                         # 任務管理層
│   ├── mission_manager.py           # 任務管理器
│   ├── waypoint.py                  # 航點定義（MAVLink Waypoint / WaypointSequence）
│   ├── survey_mission.py            # Survey 測繪任務
│   ├── coverage_path.py             # 結構化覆蓋路徑（OperationSegment / CoveragePath）
│   ├── mavlink_exporter.py          # MAVLink / QGC WPL 110 匯出（QGC / JSON / KML / GPX）
│   └── swarm_coordinator.py         # ★ 群飛協調器（SwarmCoordinator / SwarmMission）
│                                    #   - plan_coverage_dccpp(): DCCPP 完整管線入口
│                                    #   - 整合 FOV 模型、DEM 地形、PathAssembler、CoordinatedPlanner
│
├── sensors/                         # 感測器模組
│   ├── camera_model.py              # 相機模型（CameraSpecs / GSD / 覆蓋率計算）
│   ├── fov_model.py                 # ★ 梯形 FOV 地面投影模型（論文 Section 2.2.1, Eq. 2-3）
│   │                                #   - TrapezoidalFOV: 掛載角 α_m → 梯形地面投影
│   │                                #   - α_m=0 時退化為矩形 FOV（向下兼容）
│   │                                #   - compute_ground_footprint() / effective_strip_width()
│   ├── dem_terrain.py               # ★ DEM 地形雙線性插值（論文 Section 2.2.2, Eq. 4）
│   │                                #   - DEMTerrainManager: 支援 GeoTIFF (rasterio/GDAL) 及 numpy
│   │                                #   - get_elevation(lat, lon) / check_collision()
│   │                                #   - 未載入 DEM 時退化為常數海拔（向下兼容）
│   ├── sensor_fusion.py             # 感測器融合
│   └── terrain_manager.py           # 地形管理基類（TerrainManagerBase）
│
├── ui/                              # PyQt6 GUI 層
│   ├── main_window.py               # 主視窗（路徑生成協調、DCCPP 結果渲染）
│   ├── widgets/
│   │   ├── map_widget.py            # 地圖組件（Folium + WebEngine + JS 事件 + 群飛視覺化）
│   │   ├── parameter_panel.py       # 參數面板（含固定翼 / 螺旋 / 協同覆蓋 / DCCPP 區段）
│   │   ├── mission_panel.py         # 任務面板
│   │   └── polygon_editor.py        # 多邊形編輯器
│   │
│   ├── dialogs/
│   │   ├── camera_config.py         # 相機配置
│   │   ├── vehicle_config.py        # 載具配置
│   │   ├── export_dialog.py         # 匯出對話框
│   │   ├── obstacle_manager.py      # 障礙物管理
│   │   └── nfz_manager_dialog.py    # 禁航區管理
│   │
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

## DCCPP 管線架構

```
                                ┌─────────────────────────────┐
                                │   SwarmCoordinator          │
                                │   plan_coverage_dccpp()     │
                                └──────────┬──────────────────┘
                                           │
                    ┌──────────────────────┼───────────────────────┐
                    ▼                      ▼                       ▼
         ┌──────────────────┐  ┌───────────────────┐   ┌──────────────────┐
         │  TrapezoidalFOV  │  │ DEMTerrainManager │   │ RegionDivider    │
         │  (fov_model.py)  │  │ (dem_terrain.py)  │   │ decompose_by_fov │
         │  Eq. 2-3         │  │  Eq. 4            │   └────────┬─────────┘
         └────────┬─────────┘  └────────┬──────────┘            │
                  │ coverage_width       │ terrain_func          │ CoveragePath
                  └──────────┬───────────┘                      │
                             ▼                                  ▼
                  ┌─────────────────────────────────────────────────┐
                  │              DCCPPSolver (mdtsp_solver.py)      │
                  │                                                 │
                  │  Step 1: GreedyAllocator   (Algorithm 1)       │
                  │          多區域 → 多 UAV 分配                    │
                  │                                                 │
                  │  Step 2: IDPSolver         (Algorithm 2)       │
                  │          各 UAV 作業路徑序列最佳化 (MDTSP)       │
                  │                                                 │
                  │  Step 3: DCCPPPathAssembler                    │
                  │          IDP 結果 → Dubins 曲線 → 完整航點       │
                  │                                                 │
                  │  Step 4: AltitudePlanner   (Algorithm 3)       │
                  │          2D → 3D 高度剖面 + GDA 平滑             │
                  │                                                 │
                  │  Step 5: CoordinatedPlanner (可選)              │
                  │          同步多 UAV 進入時間                     │
                  └──────────────────┬──────────────────────────────┘
                                     │
                                     ▼
                          ┌─────────────────────┐
                          │   MDTSPResult        │
                          │   + assembled_paths  │
                          │     Dict[int,        │
                          │      AssembledPath]  │
                          └──────────┬──────────┘
                                     │
                          ┌──────────▼──────────┐
                          │   MainWindow UI     │
                          │   地圖渲染           │
                          │   Entry / Operation │
                          │   / Transfer 分段    │
                          └─────────────────────┘
```

## 核心類別關係圖

```
                    ┌─────────────────┐
                    │   MissionManager │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
    ┌─────────────────┐ ┌────────────┐ ┌──────────────┐
    │  GlobalPlanner  │ │LocalPlanner│ │TrajectoryOpt │
    │ (A*/RRT/Grid/   │ │   (DWA)    │ │ (Smoother/   │
    │  DCCPP/Dubins)  │ │            │ │  Dubins)     │
    └────────┬────────┘ └─────┬──────┘ └──────┬───────┘
             │                │               │
             └────────────────┼───────────────┘
                              ▼
                    ┌─────────────────┐
                    │  VehicleModel   │
                    │ ┌─────────────┐ │
                    │ │ Multirotor  │ │
                    │ │ FixedWing   │ │
                    │ │ VTOL        │ │
                    │ └─────────────┘ │
                    └─────────────────┘
```

## 技術棧

| 類別 | 技術 |
|------|------|
| 語言 | Python 3.10+ |
| GUI | PyQt6 |
| 地圖 | Folium + Leaflet + PyQt6-WebEngine |
| 地圖互動 | JS click/drag → `pyqt://` URL scheme → Python signal |
| 幾何 | NumPy, SciPy, Shapely |
| 座標投影 | pyproj + 自製 WGS84/UTM/ENU 轉換器 |
| 通訊 | pymavlink（QGC WPL 110） |
| 配置 | PyYAML |
| DEM 地形 | rasterio / GDAL / numpy（可選） |

## 論文參考

- Li et al., "Multiple fixed-wing UAVs collaborative coverage 3D path planning method for complex areas," *Defence Technology*, vol. 47, pp. 118-133, 2025.
  - GreedyAllocator (Algorithm 1), IDPSolver (Algorithm 2), AltitudePlanner (Algorithm 3)
  - 梯形 FOV 模型 (Eq. 2-3), DEM 雙線性插值 (Eq. 4)
  - 協調/非協調覆蓋模式
