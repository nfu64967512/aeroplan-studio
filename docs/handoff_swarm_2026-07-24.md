# 進度交接報告 — AeroPlan × WSL 六機固定翼編隊(2026-07-24)

> 目的:讓接手的 AI/工程師**無需先前對話脈絡**即可理解專案現況、已驗證能力、
> 架構、開放項目。所有數字皆為 WSL ArduPilot SITL 六機**實測**(非估計);
> 證據分級沿用 `[本案實測]`（SITL 真飛）/`[web:XX]`（委託文獻）/`[推測]`（工程初值）。

---

## 0. 一句話現況

六機固定翼雁行編隊在 ArduPilot SITL + ROS2 上**已跑通全流程**(冷啟動→UAV1
帶隊起飛→編隊巡航→中途 guided 轉向→長機失效替補→回同點降落),並與 Windows
的 AeroPlan 地面站串接驗證(顯示/跟隨/操控)。控制核心與韌性修復皆有 A/B 或
逐相實測背書。程式改動在分支 `fix/dccpp-geofence-2d`,commit `fc65ac8` +
`4958462`(**尚未合併 main**)。

---

## 1. 系統組成

| 層 | 位置 | 內容 |
|---|---|---|
| **地面站 AeroPlan Studio** | Windows,`python main.py`(Python 3.14 + PyQt6/QtWebEngine) | 任務規劃 GUI、2D(Leaflet)/3D(Cesium)地圖、SITL 分頁、嵌入式蜂群連線、HIL 面板 |
| **SITL** | WSL Ubuntu-22.04,`~/ardupilot` | 6× `arduplane`(直接執行 binary,非 sim_vehicle);foam `plane` / `plane-jet` 模型 |
| **編隊節點** | WSL,`~/fixwing_ws/src/sar_swarm`(ROS2 Humble) | `master_node`(UAV1)、`wingman_node`(UAV2-5)、`backup_node`(UAV6) |
| **橋接** | WSL mavproxy fan-out | 一台 SITL 分流到 MAVROS / AeroPlan / 節點三條鏈 |

**埠方案**(i=0..5):SITL master `5760+10i`、MAVROS `5860+10i`、
**AeroPlan `6760+10i`**、節點指令 `6960+10i`。
**ROS2 topic**:`/swarm/whoLeader`(長機位姿+航向+速度+omega,10 Hz)、
`/swarm/heartbeat`(master_id+health+epoch,2 Hz)、`/swarm/formation_status`
(各僚機自報 slot 誤差)、`/swarm/master_switch`(接管/讓權)。
⚠️ `/swarm/*` 為 BEST_EFFORT QoS — 訂閱端須同檔且 depth≥8(多發布者)。

---

## 2. 已完成且驗證的能力(28 項任務)

### 2.1 地面站功能
- Cesium GPU 加速(high-performance WebGL context + renderer 探測)
- HIL 連線面板(一次 6 顆飛控)、出生地預設(虎尾 / 桃園機場)
- 2D/3D 地圖預設桃園、嵌入式蜂群埠 6760/stride 10、QSS `}}` 字型洪水修正

### 2.2 WSL 分層控制
- `swarm_sitl_up.sh`(6 SITL+分流,常駐)/ `swarm_ros2_up.sh`(MAVROS+節點,可獨立起停)
- `PRESET tight/max/fast`、`MAXV=1`(泡棉解 30 m/s 參數上限)、全套編隊調參 env
- 工具:`swarm_test_report.py`(4 面板圖)、`_formation_topview.py`(耦合點雲)、
  `_ab_compare.py`、`accept_drive.py`(驗收駕駛:goto/mode/setp/land/pos)

### 2.3 控制核心(全 SITL 實測)
| 修復 | 問題 → 解 |
|---|---|
| **DO_REPOSITION 導引** | 舊 MISSION_ITEM_INT 不吃高度→長機爬 1000m+飄 11km;改 `MAV_CMD_DO_REPOSITION` 穩守高度 |
| **ramp-from-zero 閉合律** | 斜坡從 err=0 起算(舊從門檻起算→誤差漸近卡門檻、長機永不釋放) |
| **slot 前饋 + 旋轉前饋** | `wp_lead_time` 外推抵消 ~3.5s 導引滯後;弧轉時沿弧外推(直線外推會指向弧外) |
| **成形遲滯 1.8** | 巡航誤差膨脹 ~+70m;1.4→釋放即打回(巡航佔比 5%),1.8→85% |
| **§4.5 弧轉程序** | 尖角轉彎 slot 掃掠散隊;改定大半徑 fillet 弧(ω=v/R≈1.1°/s)+ 轉前減速 |

### 2.4 領導權韌性(5 修法,A/B 實測)
- 接管**資格閘**(啟動寬限 + 佐證閘,擋冷啟動誤搶/聾機劫持)
- master **降權保心跳**(舊版 deactivate 連心跳一起停→讓權死鎖=腦裂真根因)
- backup **讓權回收**(原長機健康心跳 3s→讓位)
- **DEEP_SUB_QOS**(depth 8,多發布者防自己樣本蓋掉對方)
- 代理長機**手風琴 + 直線續航**(凍結航向+前推 goto,接管後編隊重建)

### 2.5 全流程驗收航次(五相全過)
冷啟動(takeover=0)→ UAV1 guided 起飛帶隊 → t≈210s 成形巡航(err 50-120、
長機釋放 27 m/s)→ 中途 90° goto:尖峰 536→100s 重收斂 124(**有界+重收斂**)
→ `pkill master_node`:**替補 6.26s**、繞 UAV6 凝聚不散 → 停節點層 → 分層 RTL
→ 逐機 NAV_LAND 回出生點。

### 2.6 AeroPlan ↔ WSL 串接(GUI 實測)
- **顯示** ✓:6 UAV HUD(空速/高度/GS/VS/電量/GPS RTK-Fix/GUIDED ARMED/經緯度)
  + 2D 地圖 6 機標 + 各色軌跡尾
- **跟隨** ✓:即時更新(連拍 16→27 m/s、Cesium 底列追 UAV1 姿態)
- **操控** ✓:DO_REPOSITION 得 `COMMAND_ACK ACCEPTED`;Windows→WSL 6760/6810 通;
  6 client ESTABLISHED,與節點鏈並存
- **單機接管**:GCS 對任一機 RTL/AUTO,節點 standing down 不搶模式;回 GUIDED 自動歸隊

---

## 3. 關鍵實測數字

| 指標 | 數值 |
|---|---|
| 長機失效→替補 | **6.26 s**(backup timeout 6.0 + watchdog tick) |
| slot 誤差(領導修法 A/B) | mean **182 → 96 m** |
| 替補後收斂(直線續航修法) | 僚機誤差 **88-98 m**(全進 gate 120) |
| 耦合 RMS / slot 距 | S=60 **1.7-2.6×**(不耦合);S=300+§4.5 **0.67-0.83×** |
| Plan A(泡棉頂速) | 平均 **25 m/s**、巡航佔比 **85%** |
| 泡棉 vs 噴射收斂 | 泡棉 650→**51 m**;噴射卡 **~320 m** |

---

## 4. 核心工程結論(給後續設計參考)

1. **速度 vs 緊密是硬物理取捨**:轉彎半徑 r=v²/(g·tanφ)。噴射機最低可持平飛
   ~45-75 m/s → r≈500m ≫ 任何 slot 距 → 只能凝聚不能緊 V。緊 V 必須慢
   (泡棉 ~30 m/s)。**148 m/s 是機體上限、不是巡航承諾。** `[本案實測]`
2. **導引解析度下限**:GUIDED-WP 鏈間距 **S≥250-300m** 才耦合;S=60 低於 WP
   精度(~100m)→ 點雲繞長機打轉。要 60m 級緊 V 須 offboard 姿態環/INDI(見 spec R1)。
3. **編隊框應為「虛擬長機」**:slot 由框錨解算、非相對實體參考機 → 換長機不漂移。
   (spec v2.5 §3.3 已設計;現行節點尚為相對實體機,是最大單項待實作。)

---

## 5. 規格文件

`docs/formation_v6_tight_coupled_spec.md`(**v2.5**):六機緊耦合雁行編隊完整
系統需求(角色/狀態機/slot 管理/控制迴路/尾流補償/延遲容錯/驗收),含研究依據
標註。經 **9 輪驗證迴圈**(3 輪作者自檢 + 5 輪獨立多 agent 對抗審查 +
1 輪確認),**14 份 agent 審查、12 blocker + 41 major 全數修正**。附錄 A =
與現行 `sar_swarm` 落差清單(拆解任務用)。

---

## 6. 開放 / 未解決項目(接手重點)

| # | 項目 | 狀態 |
|---|---|---|
| O1 | **master_node 週期性 ~3s executor 卡頓**(疑 pymavlink TCP send 阻塞單執行緒 executor)→ 心跳缺口。現以 backup timeout 2.5→6.0 騎過;本體待修(獨立診斷任務進行中,建議 MultiThreadedExecutor + 分離 callback group) | 未解,已 workaround |
| O2 | **噴射編隊物理性鬆散 ~300m** | 固有,非 bug |
| O3 | **尾流補償(spec R5)零 SITL 驗證** | 封鎖(需 CFD+HIL) |
| O4 | **編隊框 = 虛擬長機**(spec §3.3)尚未實作於節點 | 待實作(落差清單第①) |
| O5 | SITL 固定翼 NAV_LAND 從出生點正上方起降→繞 approach 打轉;需從進場點外側直線進場(工具幾何,非控制失效) | tooling 已知解 |
| O6 | 所有 spec `[推測]` 初值(門檻係數、走廊幾何、計時器)需首飛校準 | 待校準 |
| O7 | 分支 `fix/dccpp-geofence-2d` 未合併 main | 待決 |

⚠️ **平台踩坑備忘**(避免重蹈):
- `pkill -f` 會殺自己的 shell(cmdline 含 pattern)→ 用 `pkill -x` 或字串拼接。
- WSL post-4.4 ArduPilot **參數改名**:`AIRSPEED_MAX/MIN/CRUISE`、`ROLL_LIMIT_DEG`、
  `RTL_ALTITUDE`(舊 `ARSPD_FBW_*`/`ALT_HOLD_RTL` 靜默忽略)。
- 6960 分流對**晚加入的 client 不推 telemetry** — 遙測+指令要用 6760 家族。
- SITL 需 FOREGROUND 常駐(PDEATHSIG);`nohup &` 會被 reap。
- 節點 `_keep_guided` 每 tick re-assert GUIDED — RTL/AUTO 已加白名單,可單機接管。

---

## 7. 快速重現(操作序)

```bash
# WSL:冷啟動整棧(泡棉頂速 + S=300 幾何)
bash ~/fixwing_ws/sitl_run/swarm_sitl_up.sh   # 需 MAXV=1 env(泡棉)或 JET=1(噴射)
PRESET=max V_SPACING=300 V_TAIL=600 FORMED_RADIUS=120 CATCH_RADIUS=360 \
  MANUAL=1 PATROL=1 bash ~/fixwing_ws/sitl_run/swarm_ros2_up.sh
bash ~/fixwing_ws/sitl_run/swarm_takeoff.sh          # 觸發起飛
python3 ~/fixwing_ws/sitl_run/swarm_test_report.py --duration 240 --out /tmp/r  # 出報告圖
```
```
# AeroPlan(Windows):SITL 分頁 → 嵌入式蜂群 → host=<WSL IP> port 6760 stride 10 count 6 → 連線
```

節點原始碼參考副本 + 部署指令:`scripts/wsl/sar_swarm_patches/`。
