# 蜂群系統架構 — 兩管道分工(MAVLink 飛控 / ROS2 協同)

> 設計原則(2026-07-25 確認):**Mission Planner 做得到的交給 MAVLink;
> 做不到的交給 ROS2。** 兩管道,職責不重疊。

```
                    ┌──────────────────────────────────────────────┐
                    │  ② ROS2 協同管道(MP 做不到的:多機協同)      │
                    │  sar_swarm 節點 —— 算「六台該怎麼協調」        │
                    │                                              │
                    │  formation:master/wingman/backup_node         │
                    │    · slot 幾何、leader-follower、手風琴速度    │
                    │    · 失效替補(epoch 繼任/讓權)、防撞分層      │
                    │  TOT:tot_node                                 │
                    │    · 扇形 slot 指派、共同抵達時刻 T_arr        │
                    │    · S 型機動決策、齊射時序                    │
                    │  發佈 ROS2 topic:/swarm/whoLeader、heartbeat、│
                    │    formation_status、tot_gate、tot_status      │
                    └───────────────────┬──────────────────────────┘
                          算出「去哪、多快、什麼模式」
                                        │  只經 ① 下達
                                        ▼
                    ┌──────────────────────────────────────────────┐
                    │  ① MAVLink 飛控管道(MP 做得到的:單機控制)   │
                    │  ArduPilot 自駕儀 —— 執行機體控制             │
                    │                                              │
                    │  ARM/DISARM、模式(GUIDED/AUTO/RTL/LOITER)、  │
                    │  起飛、DO_REPOSITION(goto)、DO_CHANGE_SPEED、 │
                    │  任務上傳(NAV_WAYPOINT/NAV_LAND)、參數        │
                    │  分流埠:master 5760+10i / MAVROS 5860+10i /   │
                    │    AeroPlan 6760+10i / 節點指令 6960+10i        │
                    └──────────────────────────────────────────────┘
```

## 職責歸屬對照

| 功能 | 管道 | 為什麼 |
|---|---|---|
| ARM / 起飛 / 模式切換 | ① MAVLink | 單機動作,MP 就能做 |
| 飛到某點(goto)、改空速 | ① MAVLink | `DO_REPOSITION` / `DO_CHANGE_SPEED`,單機 |
| 上傳航線 / 降落任務 | ① MAVLink | `NAV_WAYPOINT` / `NAV_LAND`,單機 |
| 讀單機遙測(位置/姿態/電量) | ① MAVLink | 自駕儀廣播,MAVROS/AeroPlan 訂閱 |
| **編隊 slot 幾何 + 誰跟誰** | ② ROS2 | 多機相對關係,MP 無此概念 |
| **長機失效 → 誰替補、slot 不散** | ② ROS2 | 分散式決策,MP 無 |
| **TOT 六台同時抵達 + S 型機動** | ② ROS2 | 機間時間對齊,MP 無 |
| **防撞 slot 指派 / 高度分層** | ② ROS2 | 全域協調,MP 無 |

## 落實現況(程式 ↔ 管道)

| 元件 | 管道 | 說明 |
|---|---|---|
| `sar_swarm/master_node`、`wingman_node`、`backup_node` | ② | 編隊協同;算 slot/失效替補,經 pymavlink 下 `DO_REPOSITION`(6960)、經 MAVROS 讀狀態 |
| `sar_swarm/tot_node` | ② | TOT 協同;算扇形/T_arr/S型機動,經 MAVLink 分流下達,發佈 `/swarm/tot_gate` |
| `scripts/wsl/accept_drive.py` | ① | MAVLink 單機手動介面(goto/mode/setp/land)—— 等同 MP 的單機動作 |
| AeroPlan「嵌入式蜂群」 | ① | 經 6760 分流讀遙測、下單機指令(顯示/跟隨/操控) |
| AeroPlan 扇形閘門疊圖 | 前端 | 規劃層:畫扇形/slot/切線(與 tot_node 同參數) |

## 互斥規則
- **編隊跟隨(master/wingman/backup)與 TOT(tot_node)不同時驅動同一批機。**
  切換 TOT 前先停編隊節點層(`pkill -x master_node wingman_node backup_node`),
  由 `swarm_tot_up.sh` 起 tot_node 獨佔。
- ① 的手動接管(GCS RTL/AUTO 單機)已加白名單:② 對該機 standing down 不搶模式,
  回 GUIDED 自動歸隊(見 `wingman_node._keep_guided`)。

## 為何這樣分(工程理由)
- **可替換性**:換飛控(ArduPilot→PX4)只動 ①;協同邏輯 ② 不變(只要吐 MAVLink)。
- **可離線測**:② 的協同演算法可用 sim GeoPose 測,不必真飛控。
- **職責單一**:① 保證單機飛安(geofence/failsafe 在飛控),② 專注多機最佳化。
- **對接 MP/AeroPlan**:任何 MAVLink 地面站都能監看/接管 ①,不需懂 ②。
