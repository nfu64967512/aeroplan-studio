# GCS 蜂群實作 → 嵌入式參考對照

> **用途**:GCS(`aeroplan-studio`)端做過的「編隊 / 長僚 / 長機替補」,給嵌入式 `~/fixwing_ws` 開發參考。
> **分工**:GCS = 遙測監看 + 上傳控制命令;**嵌入式全權群飛**。以下 GCS 邏輯多為「純邏輯參考 / 已移植政策」,
> 不是要 GCS 去管群飛——是把 GCS 驗證過的演算法與不變式,給嵌入式 raft/formation 當藍本與交叉檢查。
> **自含**:嵌入式讀不到 Windows 的 GCS 檔,故關鍵公式/不變式都直接寫在這裡。

---

## 0. 三塊對照速覽

| 主題 | GCS 檔(Windows) | 嵌入式對應 | 關係 |
|---|---|---|---|
| **編隊** | `core/swarm/biomimetic_formation.py` | `sar_swarm/sar_swarm/biomimetic_formation.py` + `wingman_node.py` | **同一個檔(md5 一致)** — 共用,非僅參考 |
| **長僚角色** | 同上(`select_leader`/`forward_reference`/`desired_slot`/`compute_velocity`) | `wingman_node` 包它 | 共用演算法 |
| **長機替補** | `core/swarm/consensus.py`(`RaftFailover`)+ `mission/failover_monitor.py` | `fleet_consensus/raft_node.py` + `sar_swarm/{master_role,backup_node}.py` | **政策已移植**;其餘為硬化參考 |

---

## 1. 編隊 biomimetic_formation.py(共用檔,嵌入式本機就有)

GCS 與嵌入式是**同一份純邏輯檔**(2D 局部 ENU 公尺座標,無 ROS/MAVLink),md5 = `3bf51eb0…`。
嵌入式可直接讀 `~/fixwing_ws/src/sar_swarm/sar_swarm/biomimetic_formation.py`。**任一邊改要兩邊同步。**

三大機制(候鳥式去中心化,取代星型「死盯長機」):
1. **相鄰單元追蹤** `forward_reference(agent, agents, goal)`:僚機不訂長機座標,而挑「比自己更靠近目標、
   且空間最近」的存活鄰機當參考點 → V 翼由局部鏈結湧現。
2. **推拉力疊加** `compute_velocity(...)`:分離(避撞 push)+ slot 吸引(pull)+ 對齊鄰機速度 +
   障礙排斥(強,觸發分裂繞行)+ 趨目標;合速度夾在 `max_speed` 內。
3. **動態長機** `select_leader(...)`:長機心跳逾時 → 距目標最近的存活機接管(Raft 風格 term+1)。
   ⚠ 注意:這裡的 leader 選法是**距目標距離**;**長機替補的「正典」已改健康度(見 §3)**,formation 這支
   的 `select_leader` 屬編隊內幾何用途,與 raft 的健康度選舉是不同層,別混。

關鍵資料結構:
- `Agent(sysid, x, y, vx, vy, is_leader, alive, last_heartbeat_t, term)`
- `FormationParams(shape='V'|'GRID'|'LINE', spacing_m, sep_radius_m, obstacle_margin_m, max_speed, w_sep, w_slot, w_align, w_obstacle, w_goal, heartbeat_timeout_s)`
- `step(agents, goal, obstacles, t_now, dt)`:選舉 → 算各機速度 → 積分位置,回傳長機 sysid。

**嵌入式接法(已實作於 wingman_node)**:鄰機 `GeoPose` → 轉 `Agent`(本地 ENU)→ `step()/compute_velocity()`
→ 取 `(vx,vy)` → **經 `send_guided_waypoint()` 下成 guided 航點(Gotcha 5,不可用 setpoint)**。

---

## 2. 長僚角色(在 biomimetic 內,可直接借鏡)

| 角色 | 行為(GCS/嵌入式同碼) |
|---|---|
| **長機 leader** | `is_leader` → 以 `w_goal*vmax` 直接趨目標領航(`compute_velocity` 的 leader 分支) |
| **僚機 wingman** | `forward_reference` 挑最近前位鄰機 → `desired_slot` 算 V/GRID/LINE 偏置 slot → 比例趨近(遠快近慢不過衝)+ `w_align` 跟鄰機同向 |
| **最前緣僚機** | `forward_reference` 回 None → 退化為直接趨目標 |

`desired_slot(agent, ref, goal)`:沿「參考機→目標」方向後退 `spacing_m`,並依 `sysid` 奇偶分左右翼
(去中心化、免全域排序);`V` 側偏 = ±spacing、`GRID` = ±0.6·spacing、`LINE` = 0。

---

## 3. 長機替補 failover —— GCS 政策(已移植)+ 可借鏡的硬化

### 3a. 已移植到嵌入式 raft_node 的「健康度政策」
GCS `core/swarm/consensus.py::health_priority_key`(min() 取最優先):
```
key = (0 if gps_ok else 1,        # GPS 正常優先
       0 if battery_known else 1, # 電量「已知」優先(未知不可冒充滿電)
       -battery_pct,              # 電量高優先
       node_id)                   # 最低 id 確定性 tiebreak
```
→ 嵌入式 `raft_node._health_frac()` 已落地(失鎖 frac=1.0、未知 0.8、否則 1-batt;commits `89f5402/0a6afb0/a19b5a9`)。
⚠ **scale 差異**:GCS `battery_pct` 用 0~100;嵌入式 `BatteryState.percentage` 用 0~1。**排序方向相同**,接的時候注意單位。

### 3b. RaftFailover 的「失效切換不變式」—— 給嵌入式 raft 交叉檢查/補強
GCS `RaftFailover`(集中式、確定性、35 個單元測試把不變式釘死)有幾個**值得嵌入式 raft_node 對照確認**的點:

1. **接管上界「設定即驗證」**:`FailoverConfig` 在建構時就檢查
   `worst = heartbeat_timeout_s + tick_period_s`,若 `worst >= takeover_deadline_s(3s)` 直接 `raise`(嚴格留餘裕)。
   → 嵌入式 raft 目前用 `election_min/max=3.0~6.0` 無此「保證 N 秒內接管」的形式化檢查;可加一個 sanity 檢查避免設定把接管拖過上界。
2. **防抖動(anti-flapping)**:健康長機只要心跳新鮮就**續任,不被更高優先序搶位**;唯有它自己失效/低健康才重選。
   → 對應嵌入式:leader 持續發心跳即續任(已有);P0-B「低健康才 stepdown」(已有)。**已對齊,可確認無「健康者一上線就搶位」的抖動。**
3. **亂序心跳單調保護**:`heartbeat(id, t)` 只在 `t > last_heartbeat` 時更新,避免遲到封包把存活者誤判失效。
   → 嵌入式走 ROS2 timer/QoS,風險低,但若有亂序遙測可參考。
4. **未知電量 ≠ 滿電**(`battery_known`):這條**直接對應**嵌入式踩到的雷——
   `BatteryState.percentage = nan`(未設 `BATT_CAPACITY`/無電流計)時**不可當滿電**,要判 `battery_known=False` → frac=0.8。
   嵌入式已照此處理,**這是兩端最重要的一致點,務必保持**。
5. **長機讓位 = 主動停心跳 + 拉長自身逾時**:`_unhealthy` 期間停發心跳、下次逾時最長,確保不立刻搶回。
   → 即嵌入式 P0-B 的作法,已對齊。

### 3c. 操作層 vs 控制面(兩層長機,刻意獨立)
GCS 早期把「選舉」與「編隊參考」混在一個 supervisor;嵌入式正確地拆成兩層,**這是比 GCS 版更好的設計,保持它**:
- **raft 選舉**(`/raft/heartbeat` → LeaderWatch)→ 控制面 HA(誰跑 coordinator/gcs_bridge)。
- **操作層 master**(`/swarm/master_switch` → 僚機編隊參考)→ 誰發 GeoPose 帶隊(master_node/backup_node)。
- 兩層各自健康度感知、互不觸發,已實測不打架。

---

## 4. 測試作為「行為規格」參考

GCS `tests/test_consensus_failover.py`(27)+ `tests/test_failover_monitor.py`(8)把 failover 不變式編成可執行規格,
嵌入式 raft 可借同樣場景補單元測試(目前嵌入式驗證偏 SITL + 離線檢查):
- 初次選舉取最健康者;健康長機續任不抖動;低健康讓位給更健康者;term 單調遞增;
- 多機同時失效;全機失聯→無長機→恢復後接管;亂序心跳被忽略;逾時邊界 inclusivity;
- **健康度決定、非距離、非角色**(對應你修掉的角色窗地雷)。

---

## 5. 一句話

編隊與長僚 = **共用同一個 `biomimetic_formation.py`**(改要兩邊同步);長機替補 = **健康度政策已移植進 raft_node**,
另有 §3b 五條失效切換不變式可供嵌入式 raft 交叉確認(其中「未知電量≠滿電」最關鍵、已對齊)。
GCS 不參與群飛決策,以上純為嵌入式的演算法藍本與規格參考。
