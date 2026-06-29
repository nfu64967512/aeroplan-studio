# 嵌入式 Handoff:健康度優先長機選舉 + 低健康備援接管

> **產生時間**:2026-06-27　**來源**:對 Jetson `nvidia@192.168.2.249` 即時勘查(非文件推斷)
> **目的**:交給新 session 執行。把嵌入式長機 failover 從「距目標距離 + 只在心跳消失時接管」
> 改成 **健康度優先選舉 + 長機健康過低即由備援接管**(2026-06-27 CEO 拍板的正典)。
> **硬規定**:嵌入式一律用 **ROS2 Humble** 開發(colcon `--symlink-install`)。

---

## 0. 連線與環境(已驗證)

| 項目 | 值 |
|---|---|
| 主機 | `nvidia@192.168.2.249`(hostname `nvidia-desktop`) |
| 登入 | **金鑰免密碼已生效**(本機 `~/.ssh/id_rsa` 已授權,`ssh nvidia@192.168.2.249` 直接通) |
| 工作區 | `~/fixwing_ws`(colcon `--symlink-install`,純 Python symlink) |
| ROS | Humble(`/opt/ros/humble`) |
| 非互動 SSH 注意 | PATH 缺 `~/.local/bin`、ROS 未 source → 遠端指令要 `export PATH="$HOME/.local/bin:$PATH"` + `source /opt/ros/humble/setup.bash` + `source ~/fixwing_ws/install/setup.bash`(或 `bash -lc`) |
| SITL | `~/ardupilot/Tools/autotest/sim_vehicle.py`(真 build);6 機由 `start_sitl_swarm.sh` 起,TCP `5760+10*i`,**無自帶 `.parm`,用 ArduPlane 預設** |

`~/fixwing_ws/src` 共 6 個 package:`sar_swarm`、`fleet_consensus`、`camera_pointing`、`height_control`、`target_tracking`、`terrain_following`(後四個與本任務無關)。

---

## 1. 嵌入式現況(即時勘查,2026-06-27)

### 1.1 版本控管狀態(重要,先處理)
- **`sar_swarm` 是 git repo**,HEAD = `3322413`:
  - `3322413 feat(swarm): operator-mode (auto_patrol) + runway lineup + DRY geodesy`
  - `d18e348 feat(swarm): 2.5s failover, layered collision-avoid, formation RTL` ← **已有一套 2.5s failover**
  - `88f278a feat(swarm): nodes fly the formation themselves via GUIDED waypoints`
  - `9e6cf60 Initial commit`
  - **未提交**:` M sar_swarm/wingman_node.py`(+131/−57)、`?? sar_swarm/biomimetic_formation.py`(從未 commit)
- **`fleet_consensus` 不在 git 控管**(`raft_node.py` 只有 `raft_node.py.orig` 可回滾)。
- `.orig` 備份:`wingman_node.py.orig`、`flight.py.orig`、`mavros_interface.py.orig`、`fleet_consensus/.../raft_node.py.orig`
- **`biomimetic_formation.py` 與 GCS `core/swarm/biomimetic_formation.py` md5 完全一致**(`3bf51eb0...`)→ **真同步,動它要兩邊一起改,別讓它漂移。**

### 1.2 目前的 failover 鏈(讀過原始碼)
```
fleet_consensus/raft_node.py   分散式 Raft 選舉(term/隨機逾時/多數決/心跳)
  ├ 偏置:_rand_timeout() 依「距 /swarm/goal 距離」加權(近→逾時短→先當選)
  ├ 訂閱:/swarm/goal、/uav{i}/mavros/global_position/global(NavSatFix,只取經緯)
  ├ 產出:/raft/heartbeat {term, leader} → 多數決後成 leader
  └ ★ 完全沒有任何健康度輸入(沒電量、沒 GPS fix 狀態)

sar_swarm/master_role.py  MasterPublisher:/uav{id}/geopose @10Hz + /swarm/heartbeat @2Hz
  └ heartbeat 內容只有 {master_id, seq, stamp}  ← 沒帶健康度

sar_swarm/backup_node.py  UAV6 空中熱備援
  └ watchdog:/swarm/heartbeat 連續 3s 沒收到 → take_over()(只看心跳消失,不看健康度)

sar_swarm/failover.py     LeaderWatch:訂 /raft/heartbeat,stale_timeout=2.5s,gate 控制輸出
```

### 1.3 健康度遙測可用性(關鍵,已確認)
slim MAVROS allowlist(`sar_swarm/config/slim_pluginlists.yaml`)= `sys_status` / `global_position` / `command` / `setpoint_*`。
→ **健康度輸入現成可用,不必加 plugin**:
- **電量**:`sys_status` plugin → `/uav{N}/mavros/battery`(`sensor_msgs/BatteryState`,`.percentage` 0~1)
- **GPS**:`global_position` plugin → `/uav{N}/mavros/global_position/global`(`NavSatFix`,`.status.status >= 0` = 有 fix)
- ⚠ **待新 session 在 swarm 運行時確認**:`ros2 topic list | grep battery`(本次勘查時 swarm 沒在跑,無法列即時 topic;SITL 是否回報電量需開起來驗一次)。

---

## 2. 目標(CEO 拍板)

> **長機選舉以「健康度」為優先;當長機健康度過低時,備用長機替代上去。**

對齊 GCS 端已實作的政策 `core/swarm/consensus.py::health_priority_key`:
**排序鍵 = (GPS 正常優先, 電量已知優先, 電量高優先, 最低 sysid)**。

⚠ **只移植「政策」,不移植「類別」**:嵌入式已有分散式 Raft,別把 GCS 的集中式 `RaftFailover` 整包搬上去(會變兩個互打的長機權威)。要做的是把這個**健康度排序政策**塞進現有 `raft_node` 的選舉偏置 + 現有 backup/master 的接管條件。

---

## 3. 要動工的項目(依優先序)

### ☐ P0-A　`raft_node.py`:選舉偏置改用健康度(取代距目標距離)
**檔案**:`~/fixwing_ws/src/fleet_consensus/fleet_consensus/raft_node.py`
1. 新增訂閱本機電量:`/uav{my_uav}/mavros/battery`(`sensor_msgs/BatteryState`),存 `self.my_batt`(0~1)。
2. NavSatFix 回呼 `on_my_fix` 已有 → 多存 `self.my_gps_ok = (msg.status.status >= 0)`。
3. 改 `_rand_timeout()`:把目前的 `frac = dist_to_goal/goal_scale` 換成健康度:
   ```python
   # 健康度越高 → frac 越小 → 逾時越短 → 越早發起選舉 → 越可能當選
   # 對齊 GCS health_priority_key:GPS 失鎖最劣;否則電量越高越優
   if self.my_gps_ok is False:
       frac = 1.0                      # 失鎖 → 最不該當長機
   elif self.my_batt is None:
       frac = 0.8                      # 電量未知 → 不冒充健康(對應 battery_known=False)
   else:
       frac = 1.0 - max(0.0, min(1.0, self.my_batt))   # 滿電→0(最短逾時)
   ```
   保留 `bias_jitter` 破除完全同時;保留「無資料 → 退回純隨機」的 fallback。
4. 把參數 `election_bias`/`goal_dist_scale_m` 旁邊加 `health_bias`(預設 True)、`low_batt_frac`(門檻)等,讓行為可調、可回退。

### ☐ P0-B　`raft_node.py`:長機健康過低 → 自動讓位(stepdown)
目前**沒有**「長機還活著但健康差就讓位」這條。加上:
1. Leader 週期自檢:若 `self.state == LEADER` 且(`my_gps_ok == False` 或 `my_batt < low_batt_threshold`,例如 0.20)→ **stepdown**:`state = FOLLOWER`、停發 `/raft/heartbeat`、並把自己下次 `_rand_timeout()` 強制拉長(`frac=1.0`)使其不會立刻又當選。
2. 心跳一停 → 其他節點選舉逾時觸發 → **最健康的存活者**(因 P0-A 偏置)勝出。
3. 這同時會餵動 `backup_node` 既有的 3s watchdog(見 P0-C),達成「長機健康過低 → 備援接管」。

### ☐ P0-C　低健康接管的「快路徑」(master_role + backup_node)
P0-B 的 stepdown 走 backup 既有 3s watchdog,但 3s 偏慢。加一條快路徑:
1. **`master_role.py`**:`_heartbeat_tick()` 的 heartbeat payload 加 `health` 欄位:
   ```python
   {"master_id": ..., "seq": ..., "stamp": ..., "health": {"batt": <0~1 or null>, "gps_ok": <bool>}}
   ```
   (MasterPublisher 需取得自身電量;`use_mavros` 模式可訂 `/uav{id}/mavros/battery`。)
2. **`backup_node.py`**:`watchdog()` 除了「3s 沒心跳」,再加「**心跳有收到、但 `health.batt < 門檻` 或 `gps_ok == False`**」→ 提前 `take_over()`。
   - 注意 `take_over()` 已會 publish `/swarm/master_switch` + activate MasterPublisher;沿用即可。

### ☐ P1-A　版本控管衛生(動 `raft_node` 前務必先做)
1. **`fleet_consensus` 沒在 git** → 編輯 `raft_node.py` 前先 `cp raft_node.py raft_node.py.bak-$(date)`(已有 `.orig` 但那是舊改動的)。建議把 `fleet_consensus` 也 `git init` 納管。
2. `sar_swarm` 有未提交的 `wingman_node.py`(+131/−57)與未追蹤的 `biomimetic_formation.py` → **先 commit 這些「已部署未提交」的 2026-06-27 仿生升級**,讓基準乾淨,之後 failover 改動才好 review/回滾。
   - ⚠ `biomimetic_formation.py` 與 GCS 同檔(md5 一致),commit 時兩邊保持一致。

### ☐ P1-B　實飛驗證 2026-06-27 仿生升級(嵌入式節點版本「從未實飛」)
文件記載的「V 成形→分裂→殺長機接管」三相實測,**是 GCS 端執行器(`%TEMP%/formation_sitl.py`)驅動的,不是嵌入式 `wingman_node` 自己飛的**。嵌入式節點版本需在 SITL 監督下實飛一次。

---

## 4. 驗證計畫(SITL,新 session 必跑)

1. **起 swarm**(固定順序,各佔一終端,先 source 環境):
   `start_sitl_swarm.sh` → `bridge_mavproxy_swarm.sh` → `start_mavros_swarm.sh` → `ros2 launch sar_swarm sar_failover_nodes.launch.py use_mavros:=true`
2. **先確認健康 topic 真的有**:`ros2 topic echo /uav1/mavros/battery --once`(確認 SITL 回報電量;若 `percentage` 為 nan/0,需在 SITL 設 `SIM_BATT_VOLTAGE`/`BATT_MONITOR` 或改用 SYS_STATUS.battery_remaining)。
3. **健康度選舉**:讓某機電量較低(SITL 調 `SIM_BATT_*` 或在節點注入),確認 Raft 選出的 leader 是**最健康者**,而非最低 id / 最近目標者。
4. **低健康讓位**:把現任長機(UAV1)電量壓到門檻以下 → 確認其 stepdown、`/swarm/master_switch` 切到健康者、`backup_node`(UAV6)接管並發 GeoPose,wingmen rebind 重組。量「健康過低 → 接管完成」時間。
5. **回歸**:確認原本「心跳消失」的接管路徑沒被破壞(殺掉 master process,backup 仍 3s 內接管)。

---

## 5. 新 session 必須記住的限制 / 雷區

- **ROS2 only**:所有改動在 `~/fixwing_ws` 內,`colcon build --symlink-install` 後生效;node 改完要 rebuild + 重起。
- **Gotcha 1(SSH PATH/source)**:見 §0。
- **Gotcha 4(mavproxy 單 client)**:bridge 每個 `tcpin` 埠只服務一個 client,別重複連同一埠。
- **Gotcha 5(GUIDED 24s 硬牆)**:固定翼 GUIDED 不吃 `SET_POSITION_TARGET`,所有移動指令走 `mavros_interface.send_guided_waypoint()`(`MISSION_ITEM_INT current=2`,~24s 固有延遲,不能 spam,`wp_interval≈25s`)。**本任務只動 failover/選舉,不碰這條,但別誤改成 setpoint。**
- **biomimetic 同步**:`sar_swarm/biomimetic_formation.py` == GCS `core/swarm/biomimetic_formation.py`(md5 一致),要改一起改。
- **回滾**:`fleet_consensus` 無 git,改 `raft_node.py` 前先手動備份;`sar_swarm` 有 git + `.orig`。
- **電量在 SITL**:需確認 SITL 真的回報電量(見 §4.2),否則健康度政策的電量維度無效,只剩 GPS-fix 維度。

---

## 6. GCS 端參考(已完成,可照抄政策)

- 政策實作:`aeroplan-studio/core/swarm/consensus.py`
  - `health_priority_key(node)` → `(0 if gps_ok else 1, 0 if battery_known else 1, -battery_pct, node_id)`
  - `RaftFailover`(集中式監督者,**不要**整包移植)、`FailoverConfig`(3s 接管上界驗證)
  - 35 個單元測試:`tests/test_consensus_failover.py`、`tests/test_failover_monitor.py`(均**未 commit**)
- GCS `FailoverMonitor`(`mission/failover_monitor.py`)是 GCS 端觀測者/監督者,**非實機長機權威**;實機權威仍是嵌入式 `raft_node`。兩端對齊靠這份文件把健康度政策落到 `raft_node`。

---

## 7. 一句話總結

嵌入式的 failover **已能跑、且比文件記載更新**(已有 2.5s failover),但 **選舉只看距離、接管只看心跳消失,兩者都沒有健康度維度**。健康遙測(電量/GPS)現成可用、不必加 plugin。要做的就是 §3 的 **P0-A/B/C**:把 `raft_node` 選舉偏置改健康度、加長機低健康自動讓位、讓 backup 在長機健康過低時提前接管——全部在 ROS2 內、可單機 SITL 驗證、可回滾。
