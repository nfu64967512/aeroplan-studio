# AeroPlan Studio — 專案現況、架構與底層打通

> 產生日期：2026-06-27 ｜ 分支：`refactor/strike-arch-rework-2026`
> 本文件為「快照式」進度總覽：專案架構、已打通的底層、SITL 模擬環境、Jetson `fixwing_ws` 建置階段。

---

## 1. 專案概述

**AeroPlan Studio** 是一套以 PyQt6 開發的無人機地面控制站（GCS）＋ 任務規劃 ＋ SITL 模擬整合環境，目標支援固定翼 / VTOL / 多機蜂群，全 UI 遵循 **MIL-STD-1472H** 人因規範。

| 面向 | 技術 |
|------|------|
| GUI | PyQt6 + QWebEngine |
| 地圖 | Cesium（3D 地形 / 姿態模型）＋ Leaflet（2D） |
| 通訊 | MAVLink（pymavlink）— TCP 直連 SITL / 經 mavproxy bridge |
| 嵌入式 | NVIDIA Jetson（ROS2 Humble）跑 ArduPlane SITL + sar_swarm 蜂群節點 |
| 規劃 | A*/RRT/Dijkstra、DCCPP 覆蓋、Dubins/spline 軌跡、菱形蜂群打擊、仿生編隊 |

---

## 2. 整體架構（分層）

```
┌─────────────────────────────────────────────────────────────┐
│  ui/        PyQt6 GCS：main_window + controllers + widgets    │  表現層
│             + Cesium/Leaflet 地圖 + STOT 儀表板                │
├─────────────────────────────────────────────────────────────┤
│  mission/   MAVLink/SITL 鏈路、FleetRegistry 共享黑板、         │  整合層
│             swarm_coordinator、geofence、mission_export        │
├─────────────────────────────────────────────────────────────┤
│  core/      純演算法（無 Qt / 無 MAVLink，可單元測試）：        │  演算法層
│             strike / swarm / dccpp / global_planner /          │
│             local_planner / trajectory / geometry / vehicles   │
└─────────────────────────────────────────────────────────────┘
        │ TCP MAVLink (6760-6810 經 bridge，或本機 5760)
        ▼
┌─────────────────────────────────────────────────────────────┐
│  Jetson fixwing_ws（ROS2 Humble）：ArduPlane SITL ×6 +         │
│  sar_swarm（master/wingman/backup）+ fleet_consensus（Raft）   │
└─────────────────────────────────────────────────────────────┘
```

**設計原則**：演算法（`core/`）刻意與 Qt/MAVLink 解耦 → 可 headless 單元測試、可重用、可部署到嵌入式。整合層（`mission/`）負責把演算法接到實際飛控與 UI。

---

## 3. 程式碼架構（目錄）

頂層：`core/ mission/ ui/ config/ cli/ sensors/ sitl/ scripts/ tests/ docs/ assets/ data/`

### core/ — 演算法層
| 模組 | 內容 |
|------|------|
| `core/strike/` | 蜂群打擊：`diamond_swarm_planner`（菱形 STOT/DTOT 規劃）、`terminal_sync*`（閉環終端同步）、`tgo_coordination`（t_go 協商）、`tot_controller`（ToT 速度修正）、`maneuver_path`（S 機動）、`swarm/vtol/terminal` 各式 planner、`dtot/time_coordination`、`mission_export` |
| `core/swarm/` | `biomimetic_formation`（仿生編隊：相鄰追蹤＋推拉力＋動態長機） |
| `core/global_planner/` | A* / RRT / Dijkstra / 覆蓋路徑 / NFZ / mdTSP / 固定翼 planner / grid |
| `core/local_planner/` | APF / DWA / MPC |
| `core/dccpp/` | 分散式協同覆蓋路徑（task allocation、collision avoidance、idp solver、path builder） |
| `core/trajectory/` | Dubins / spline / 平滑 / 時間最佳 / DCCPP 組裝 |
| `core/geometry/` `core/base/` `core/collision/` `core/vehicles/` | 座標/多邊形、固定翼約束、碰撞檢查、fixed_wing/multirotor 模型 |

### mission/ — 整合層（關鍵檔）
`sitl_link`（單機 MAVLink QThread）、`fleet_registry`（多機共享黑板）、`swarm_coordinator`、`geofence_manager`、`mission_manager`、`mavlink_exporter` / `vtol_mission_exporter`、`coverage_path`、`survey_mission`、`param_service`、`sitl_launcher`、`mock_sitl_link`。

### ui/ — 表現層
`main_window.py`（主視窗，多機 SITL 鏈路管理）、`controllers/strike_controller.py`（蜂群打擊全部 handler）、`widgets/`（dual_map、cesium、sitl_hud、drone_detail_panel、strike_ttt_dashboard…）、`dialogs/`、`resources/`（tactical_theme QSS、aeroplan_theme）、`widgets/cesium_templates/main.html`。

---

## 4. 已打通的底層（Foundations）

### 4.1 MAVLink / SITL 鏈路層 ✅
- **`SITLLink`**（`mission/sitl_link.py`）：每機一條 TCP MAVLink，背景 QThread 收遙測 → `TelemetryFrame`（lat/lon/alt/姿態/速度/電量/模式…），主執行緒以命令佇列下令：`arm/disarm/set_mode/guided_takeoff/guided_goto/change_speed/upload_mission/auto_start/set_param/upload_fence/...`。多來源遙測自動依 sysid 分流。
- 固定翼相容處理：GUIDED 起飛走 ArduPlane **TAKEOFF 模式（id 13）**；「飛到」走 **`MISSION_ITEM_INT current=2`** 的 guided 航點（純固定翼 GUIDED 不吃 `SET_POSITION_TARGET`）。

### 4.2 FleetRegistry 共享黑板（資訊互通）✅
- **`FleetRegistry`**（`mission/fleet_registry.py`）：單例，集中全機最新 `TelemetryFrame`，`telemetry_updated` 廣播；`snapshot()` / `latest_by_sysid()` 提供「全機即時態勢」單一讀取點。
- 這是**各機之間資訊互通**的骨幹：打擊/編隊執行器每個 tick 讀全機快照，每架的指令都是全機狀態的函數（同步釋放閘、ToT 速度、兩兩防撞）。

### 4.3 蜂群打擊執行（雙模式）✅
- **開環 KAMIKAZE**：`diamond_swarm_planner` 規劃每機完整任務（並排起飛→編隊巡航→決斷圈四散→S 機動/盤旋補時→多向俯衝）→ 上傳 SITL → `launch_kamikaze_synchronized` 依 `takeoff_delay_s` 排程啟動。
- **閉環 TERMINAL SYNC**（本期新增）：`terminal_sync_coordinator` 以 FleetRegistry 為態勢源，三段式 STAGE→RELEASE→CORRECT：
  - **t_go 協商**（`tgo_coordination`）：協商「整體能量最省」的共同命中時刻 t\*（協同變數，黃金分割求解）。
  - **牧羊犬網格**：遠機全速直線衝刺、近機 **S 型機動**（航向偏角 θ=acos(v_close/v_act)，閉環對時）耗時，收斂同一 t\*。
  - **ToT 速度修正**（`tot_controller`）。
  - 實機散度：等距環 **~1–2s**（開環 ~38s → 閉環 ~1s）。

### 4.4 仿生編隊（去星型）✅（演算法 + 嵌入式部署，未實飛）
- **`core/swarm/biomimetic_formation`**（GCS）＝ Jetson `sar_swarm/biomimetic_formation`（同檔）：相鄰單元追蹤（追最近前位鄰機，非長機）＋ 推拉力（分離/slot/對齊/障礙/趨目標）＋ 動態長機選舉（距目標最近者接管）。
- GCS 端執行器對 live SITL 三相實測通過：**V 成形（leader 在前、minSep>300m）/ 障礙分裂（最近 218m>150m 半徑）/ 殺長機→UAV2 接管**。

### 4.5 其他既有底層 ✅
電子圍籬自動建構（`geofence_manager`，每條路徑綁 4 頂點矩形 Geofence）、QGC/MP 任務匯出、DCCPP 多機覆蓋、Cesium 3D 姿態（pitch/roll/yaw）、2D/3D 雙圖右鍵 GUIDED + 高度框、SITL HUD/Servo/Command 分頁。

---

## 5. SITL 模擬環境

### 5.1 本機 Windows SITL（`sitl/`）
內含 `ArduPlane.exe` / `ArduCopter.exe` + `sim_vehicle.py` + `models/` + cygwin DLL → 可在 Windows 本機起 SITL（開發/單機測試用）。

### 5.2 Jetson SITL 蜂群（主力，見 [[jetson-swarm-runtime]] 記憶）
6 架 ArduPlane SITL @ `192.168.2.249`，三條腳本固定順序：
1. `start_sitl_swarm.sh` — 6 架 SITL，TCP **5760–5810**（單一 TCP client）。次要埠 **5762/5772/.../5812** 開在 0.0.0.0、可直連（測試/腳本用，不被 bridge 佔）。
2. `bridge_mavproxy_swarm.sh` — 每機 mavproxy fan-out：MAVROS **5860–5910** ＋ AeroPlan **6760–6810** ＋ 節點 **6960–6810**。
3. `start_mavros_swarm.sh` — 6 個 mavros_node。

**AeroPlan 接法**：TCP client 連 `tcp:192.168.2.249:6760..6810`（sysid 1..6），**非 5760**。UI 走 SITL HUD「嵌入式蜂群」按鈕 → `EmbeddedSwarmDialog`。

> ⚠️ **Gotcha 5（硬牆）**：ArduPlane 固定翼 GUIDED **不執行 `SET_POSITION_TARGET`**，唯一能動的是 `MISSION_ITEM_INT current=2` guided 航點，且有 **~24s 固有延遲**（直連也有）。故即時僚機編隊收斂慢、間距偏寬。換韌體（4.6.3 / 4.8-dev）行為相同。

---

## 6. Jetson 環境架構與程式碼架構

### 6.1 系統環境
NVIDIA Jetson @ `192.168.2.249`（user `nvidia`）｜JetPack 6.2 / L4T R36.4.7｜ROS2 Humble｜工作空間 `~/fixwing_ws`（colcon，`--symlink-install`）。

### 6.2 執行期架構（runtime data flow）
三條腳本固定順序起整套模擬（埠對照見 §5.2）：

```
  ArduPlane SITL ×6  (TCP 5760-5810, 單一 client)
        │ MAVLink
        ▼
  mavproxy bridge  (bridge_mavproxy_swarm.sh, 每機 fan-out 三路)
   ├─► MAVROS 埠 5860-5910 ──► mavros_node ×6 ──► /uav{i}/mavros/* (ROS2)
   ├─► AeroPlan 埠 6760-6810  (GCS 監看遙測)
   └─► 節點 link 埠 6960-6810 (mavros_interface.send_guided_waypoint
                               走 pymavlink 送 MISSION_ITEM_INT current=2)
        │
        ▼  ROS2 graph (sar_failover_nodes.launch.py)
  master_node(UAV1) ─/uav1/geopose@10Hz, /swarm/heartbeat@2Hz─┐
  backup_node(UAV6, 空中熱備援, 同 MasterPublisher idle→activate)│
  raft_node ×6 ─/raft/{heartbeat,vote_*}─► 選舉 → /swarm/master_switch
                                                          ▼
  wingman_node(UAV2-5): 訂閱「全機」/uav{j}/mavros/global_position/global
        → biomimetic 相鄰 slot → send_guided_waypoint(慢節奏, 撞 Gotcha 5 ~24s)
```

**ROS2 topic 對照（節錄）**

| Topic | 發布 → 訂閱 | 用途 |
|-------|-----------|------|
| `/uav{i}/geopose` | master/backup → wingman | 長機定位（SIM 模式 / 長機基準） |
| `/uav{i}/mavros/global_position/global` | mavros_node → wingman/raft | 全機經緯（相鄰追蹤、距目標偏置輸入） |
| `/uav{i}/mavros/setpoint_raw/global` | wingman → mavros | 編隊目標 setpoint（實走 guided WP） |
| `/swarm/master_switch` | 選舉 → wingman | 長機切換（rebind 參考） |
| `/swarm/heartbeat` / `/swarm/master_state` | master → 全隊 | 長機心跳 / 狀態 |
| `/swarm/rtl` | GCS → wingman | 編隊 RTL 切換 |
| `/swarm/obstacles` | (外部) → wingman | 障礙清單（觸發分裂/重組） |
| `/swarm/goal` | (GCS/任務) → raft | 目標座標（驅動「距目標最近者接管」偏置） |
| `/raft/heartbeat` / `/raft/vote_request` / `/raft/vote_reply` | raft ↔ raft | Raft 選舉協定 |
| `/swarm/detections` / `/swarm/drop_assignments` / `/swarm/phase` | 感測/協調 | SAR 任務協同 |
| `/swarm/tx` / `/swarm/rx` / `/gcs/downlink` | link_layer / gcs_bridge | 有損鏈路模擬 / GCS 下行 |

### 6.3 fixwing_ws 套件總覽（6 個）
| 套件 | 用途 | 節點 / 模組 |
|------|------|-----------|
| **sar_swarm** | 固定翼蜂群任務編排（ROS2 + ArduPilot SITL） | 9 個節點（見 §6.4） |
| **fleet_consensus** | Raft 長機高可用選舉 | `raft_node`、`delivery_scheduler` |
| **camera_pointing** | FOV 受限俯仰控制，鎖定固定地面 landmark | `pointing_node`、`fov_controller` |
| **height_control** | 固定翼測繪定高（15m AGL，MPD 控制器） | `kf` 類控制庫 |
| **target_tracking** | IMM/MMAE 視覺追蹤機動地面車輛 | `tracker_node`、`imm_tracker`、`mmae_estimator` |
| **terrain_following** | 3-state Kalman 融合下視雷達/雷射高度計 + IMU | `kf_node`、`altitude_kf` |

### 6.4 sar_swarm 程式碼架構
**ROS2 節點（entry points）**
| 節點 | 職責 |
|------|------|
| `master_node` | 長機：起飛後飛巡邏方框（1.5km box）帶隊，`MasterPublisher` 發 GeoPose@10Hz + heartbeat@2Hz |
| `wingman_node` | 僚機：**仿生相鄰追蹤**（訂全機 global fix，挑最近前位鄰機算 V slot，障礙推離；SIM 退回原星型） |
| `backup_node` | 空中熱備援：idle 的 `MasterPublisher`，watchdog 觸發後 activate 接管 |
| `swarm_coordinator` | 蜂群任務協調 / 分派 |
| `vehicle_agent` | 單機載具代理（狀態/介面） |
| `link_layer` | 有損無線鏈路模擬（`/swarm/tx` `/swarm/rx`） |
| `eo_ir_sensor` | EO/IR 感測 → `/swarm/detections` |
| `gcs_bridge` / `gcs_monitor` | GCS 橋接 / 監看（`/gcs/downlink`） |

**支援模組（library，非節點）**
`master_role`（MasterPublisher：GeoPose+heartbeat）、`failover`（LeaderWatch：訂 `/raft/heartbeat` gate 輸出）、`flight`（固定翼起飛 FSM：TAKEOFF 模式 id 13 滑跑爬升）、`mavros_interface`（MAVROS 狀態/arm/mode + `send_guided_waypoint` pymavlink）、`mavros_launch`、`formation_control`（沿航速度 PI，沿/橫切誤差分離）、`mission_config`、`geodesy`（共用 dist_m / EARTH 常數）、`biomimetic_formation`（**新**，與 GCS `core/swarm/biomimetic_formation` 同檔）。

### 6.5 fleet_consensus — Raft 選舉
`raft_node`：實作 Raft 選舉半（terms / 隨機 election timeout / RequestVote 多數決 / leader heartbeat），不含 log replication。**2026-06-27 升級**：election timeout 依「距 `/swarm/goal` 距離」加權 → 最近者最先超時、最先當選（牧羊犬接管；無 goal 退回純隨機）。

### 6.6 啟動與腳本
- **腳本**（`sar_swarm/scripts/`）：`start_sitl_swarm.sh`（6 SITL）→ `bridge_mavproxy_swarm.sh`（fan-out）→ `start_mavros_swarm.sh`（mavros×6）。
- **launch**（`sar_swarm/launch/`）：`apm_sitl{1-6}.launch.py`（單機 SITL）、`mavros_swarm.launch.py`、`sar_failover_nodes.launch.py`（master+wingman+backup+raft 一鍵）、`swarm_failover.launch.py`、`swarm_sitl.launch.py`。
- 一鍵起編隊：`ros2 launch sar_swarm sar_failover_nodes.launch.py use_mavros:=true`（需先 source ROS + ws + `~/.local/bin` PATH，見 [[jetson-swarm-runtime]] Gotcha 1）。

### 6.7 建置 / 部署狀態
- **建置**：`colcon build --symlink-install`（純 Python symlink）；`sar_swarm` + `fleet_consensus` build 乾淨、節點 import + 演算法在 Jetson 實測通過。
- **2026-06-26（commit 88f278a）**：蜂群改自用 guided 航點飛（繞 Gotcha 5），實測 6 架起飛→V 成形→殺 master 後 UAV6 接管重組。
- **2026-06-27 仿生編隊升級**：`wingman_node`（相鄰追蹤）+ `raft_node`（距目標偏置）+ 新 `biomimetic_formation`；`.orig` 備份都在。
  > **狀態：已部署、已建置、import/演算法 OK ——但尚未實飛、尚未 commit 嵌入式 repo。** 上實機前需在 SITL 監督飛測；要回滾隨時 `cp *.orig` 還原再 build。

---

## 7. 本期（strike-arch-rework）完成清單

| commit | 內容 |
|--------|------|
| `7ef5e6d`/`65ec948` | S 機動補時、launch_stagger（跑道安全） |
| `996c90c` | ToT 速度修正 + SOLID/KISS refactor + FormationConfig fail-fast |
| `19e7f59` | terminal_sync 幾何（等距 push point + 同步釋放） |
| `5d591ac` | 閉環終端同步 STOT 執行器接進 live GCS（FleetRegistry 互通） |
| `9f8351b` | t_go 協同變數協商 + 牧羊犬網格終端制導 |
| `0501428` | 對抗式審查 13 項加固 |
| `73c63f3` | S 型機動終端制導（取代盤旋） |
| `304a756` | 仿生編隊演算法（去中心化推拉） |

實機驗證里程碑：閉環 STOT 同時命中 ~1–2s、6 機繞開無碰撞；仿生編隊 V/分裂/接管三功能對 live SITL 全過。

---

## 8. 已知限制與待辦

- **Gotcha 5（ArduPlane guided-WP ~24s 延遲）**：即時僚機編隊收斂慢、V 偏寬。要更緊湊需在 ArduPlane 端處理延遲。
- **仿生編隊嵌入式**：已部署但**未實飛、未 commit**。需 SITL 監督飛測後再 commit。
- **閉環 STOT 極端落差**：等距環緊（~2s），極端不等距（700 vs 2600m）受 GUIDED carrot 跟隨保真度限制偏鬆（~10–48s）；實際包圍環場景不受影響。
- **仿生編隊 UI**：GCS 執行器已可對 SITL 實跑，但尚未做成 AeroPlan UI 按鈕（可比照打擊執行器接 `SITLLink`+`FleetRegistry`+地圖視覺化）。
- **refactor 分支 WIP**：本分支尚有 ~100 個檔案處於重構中（whole-project DRY/KISS/SOLID，見 `docs/refactor_audit_2026.md`）。
- **SSH 免密碼**：屬存取控制變更，由使用者自行設定（`ssh-copy-id`），AeroPlan 端續用密碼 askpass 連 Jetson。
