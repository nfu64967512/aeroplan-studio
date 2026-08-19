# STOT 六機齊射 —— 系統架構與完整流程

> 2026-07-29。SITL(ArduPilot)+ ROS 2 Humble(sar_swarm)雙層架構。
> 搭配 `stot_status_2026-07-29.md`(成果與問題)閱讀。

---

## 1. 架構分工

系統刻意切成**兩層**,職責不重疊。這個切法是整個專案的地基,
違反它就會出現「兩個控制器搶同一架飛機」那類問題。

| 層 | 負責 | 實作 | 介面 |
|---|---|---|---|
| **① 飛控層** | **單機**怎麼飛到被指定的點 | ArduPilot SITL(`arduplane`) | MAVLink |
| **② 協同層** | **多機**之間的相對關係與時序 | ROS 2 `sar_swarm` | ROS 2 topics |

**判準**:一件事只牽涉一架飛機 ⇒ ①;牽涉「這架相對那架」⇒ ②。

- 「飛到 (lat,lon,alt) 並保持 135 m/s」→ ① TECS + L1
- 「排在長機後方 1056 m、左偏 35°」→ ② `wingman_node` 算出座標,再交給 ①
- 「六機同時抵達紅心」→ ② 決定各機時刻表,① 執行速度指令

⚠️ **同一個受控量只能有一個權威。** 本專案踩過的實例:
`jet_show_ros2.py` 與 `master_node` 同時對 UAV1 發 `DO_CHANGE_SPEED`,
兩者交替覆蓋、誰最後送誰贏。現以 `SHOW_LEAD_SPEED=0` 讓 `master_node` 獨佔速度,
腳本只管航點。

---

## 2. 進程與埠位圖

```
Windows 11
  └─ X-Plane 12 / Mission Planner / QGC / AeroPlan Studio
        ▲ udp:14550+i   (唯讀觀測;GCS_HOST 設定才啟用)
        │
════════╪════════ WSL2 邊界(NAT:WSL 出得去、Windows 連不進來)════════
        │
WSL2 Ubuntu 22.04
  │
  ├─ arduplane × 6                    SITL 本體
  │     └─ tcp:5760+10i               MAVLink 主鏈(每機)
  │
  ├─ mavproxy.py × 6                  一機一組分流
  │     --master=tcp:127.0.0.1:5760+10i
  │     --out=tcpin:5860+10i   →  MAVROS(ROS 2 層)
  │     --out=tcpin:6760+10i   →  表演腳本 / AeroPlan
  │     --out=tcpin:6960+10i   →  tot_node
  │     --out=udpout:$GCS_HOST:14550+i  →  Windows 端(選配)
  │
  └─ ROS 2 Humble
        ├─ mavros × 6                 /uav{i}/mavros/**
        ├─ master_node                UAV1
        ├─ wingman_node × 4           UAV2–5
        ├─ backup_node                UAV6
        └─ tot_node                   終端 STOT(直連 6960+10i,不走 MAVROS)
```

⚠️⚠️ **分流必須分家。** MAVProxy 的每個 `--out` 是獨立連線,
但**同一個 out 只能有一個消費者** —— 先連者獨佔遙測,後連的讀不到位置。
這是本專案的招牌靜默失敗之一。

| 埠 | 消費者 | 備註 |
|---|---|---|
| 5760+10i | mavproxy master | 不要直接接 |
| 5860+10i | MAVROS | ROS 2 層專用 |
| 6760+10i | `jet_show_ros2.py` / AeroPlan | 表演腳本 |
| 6960+10i | `tot_node` | 終端導引專用 |
| 14550+i | X-Plane / 地面站 | `GCS_HOST=<Windows IP>` 才啟用 |

取 Windows IP:`ip route show default | awk '{print $3}'`

---

## 3. ROS 2 節點與 topic

### 3.1 節點

| 節點 | 機號 | 職責 |
|---|---|---|
| `master_node` | UAV1 | 發布 `whoLeader`、**Max-Consensus 速度對齊**、(選配)自動巡邏 |
| `wingman_node` | 2–5 | 依長機航向算極座標 V slot,下 `DO_REPOSITION` |
| `backup_node` | 6 | V 尾;長機失效時接管領導權 |
| `tot_node` | 全隊 | 終端 STOT:扇形 → 環 → 俯衝(**不經 MAVROS**) |

### 3.2 Topic

| Topic | 型別 | 內容 |
|---|---|---|
| `/swarm/whoLeader` | `String` | `{id, lat, lon, alt, heading, speed}` @ 10 Hz |
| `/swarm/heartbeat` | `String` | 各機健康度(選舉用) |
| `/swarm/master_switch` | `String` | 領導權轉移 |
| `/swarm/rtl` | `Bool` | 編隊返航 |
| `/swarm/obstacles` | `String` | 障礙(biomimetic 模式用) |

⚠️ `whoLeader` 由 `MasterPublisher._geopose_tick` 的**獨立 10 Hz timer** 發布,
**不經過** `master_node._tick` 的 `auto_patrol` early return。
所以 `MANUAL=1`(operator 模式)之下 `whoLeader` 照發 —— 這點曾被我誤判過。

### 3.3 V 隊形幾何

```
V_RANK_SIDE = {2:(1,+1), 3:(2,+1), 4:(1,-1), 5:(2,-1)}
distance    = rank × v_spacing_m
bearing_off = 180° + side × v_angle_deg      (相對長機航向)
```

`v_spacing_m=1056`、`v_angle_deg=35` 之下:

| 機 | 沿航向 | 垂直航向 |
|---|---|---|
| UAV2 | −865 m | −606 m |
| UAV3 | −1731 m | −1212 m |
| UAV4 | −865 m | +606 m |
| UAV5 | −1731 m | +1212 m |
| UAV6 | `v_tail_dist_m` 正後方 | 0 |

另有 `formation_shape=CONE` 的立體錐形(`CONE_PHASE_DEG`,含垂直分量),
已實作但尚未跑過對比航次。

---

## 4. 完整任務流程

```
① 彈射起飛        jet_catapult_launch.py
      ↓            LAUNCH_MODE=guided(不經 TAKEOFF 模式)
                   ⚠️ 必須先給導引目標再解鎖
                   TKOFF_THR_BOOST=60 → 起飛後收回 CRUISE_THR_MAX=21
      ↓            六機依序彈射(STAGGER)⇒ **3.2 km 初始缺口**
                   
② 爬升            CLIMB_TO_ALT=3100 / CLIMB_TO_V=96
      ↓            96 m/s 是最佳爬升速度 √(T/3c),RoC 25.7 m/s
                   ⚠️ 必須在 ROS 2 編隊層接手**之前**完成
                   (僚機每秒重發航點會重錨高度目標)
                   
③ ROS 2 編隊巡航   jet_show_ros2.py(驅動長機航點)
      ↓            + wingman_node(僚機守 slot)
                   長機航點永遠放在「當前位置往紅心再推 LEAD_AHEAD」
                   ⚠️ 固定翼停不住 —— 給定點就會繞圈
                   速度由 master_node 的 max-consensus 決定
      ↓            交棒:距紅心 HANDOFF_D = 40 km
                   
④ ALIGN           tot_node:六機散進 35 km 扇形(半角 25°,6 slot)
      ↓            沿各自徑向筆直向內,用 **weave(S 型機動)** 吸收時間差
                   weave_period=120 s(157 m/s 的轉彎率只有 3.58°/s)
                   ⚠️ TOT 必須在這一段決定完,不可在目標前方盤旋
                   
⑤ PERCH          抵達 pushover 環(27° 之下約 9691 m)
      ↓            環半徑由**實測地形**重算(_reground)
                   perch_settle=5 s ≈ 不等
                   
⑥ DIVE           27° 俯衝,PN 接管整段(GATE_R 在環內側)
      ↓            PN_LOS_FLOOR=1.0:指令俯角不准比直指紅心的視線更淺
                   PN_NZ_MIN=−0.2:允許小負 g 推頭
                   50 Hz SET_ATTITUDE_TARGET(繞開 TECS 與 L1)
                   
⑦ 命中            impact_reach=70 m 或通過最近點
```

### 4.1 tot_node 的階段機

```
INIT → ALIGN → PERCH → DIVE → DONE
        │        │       │
        │        │       └─ 放行閘門(三選一):
        │        │            (a) 時刻表:d/pace ≥ dive_T − now
        │        │            (b) 幾何守門員:atan(h/d) ≥ dive_angle_max
        │        │            (c) 環閘門:必須 d ≤ pushover_r × 1.10
        │        └─ 全員抵環 → 高度收斂(perch_band)或 perch_settle 逾時
        └─ 全員抵扇形 slot(reach)且橫偏 < align_tol
```

⚠️ **環閘門是必要的**。沒有它,DIVE 一開始(蜂群還在 35 km)時刻表就把全隊放掉,
而從 34 km 瞄紅心的下滑線只有 10° —— 環等於不存在。

---

## 5. 關鍵演算法

### 5.1 Max-Consensus 速度對齊(`master_node`)

```python
v_lead = clamp(catch_speed − worst / close_time, wait_speed, nominal_speed)
```

`worst` = 最差僚機對自己 slot 的位置誤差。

**為什麼不是二值切換**:低推重比固定翼的速度權限**不對稱** ——
加速靠剩餘推力(+21 EAS),減速靠阻力(**−45 EAS**,加速度快 2 倍)。
只讓落後機加速要 185 km 航程才能修正 167 s 散布;兩邊同時用只要 41 km。

⚠️ **配套**:`wait_speed` 必須取巡航的 **0.67 倍**(90 EAS)而非對稱的 0.84 ——
取對稱下緣會讓 max-consensus 在大落後時退化成和舊二值切換一樣(閉合率同為 22 m/s)。
實測閉合率 **53 m/s**(UAV6 在 188 s 內收掉 10 km)。

### 5.2 終端比例導引(`tot_node._pn_cmd`)

三通道解耦,直接下 50 Hz 姿態指令:

```
橫向  λ̇ = (de·vn − dn·ve)/d²        roll = atan2(N·vh·λ̇, g·cos γ)
縱向  σ̇ → γ̇ = −N_vert·σ̇            nz ∈ [pn_nz_min, 3.0]
      ⭐ γ_cmd = min(γ_pn, γ_los)     ← 幾何前饋,不准比視線淺
油門  v_req = R/t_rem                thr = trim + gain·(v_req − V)
```

⚠️ **`γ_cmd = min(γ_pn, γ_los)` 是陡俯角的真開關**,不是 `dive_angle`。
純 PN 的縱向通道完全不引用 `dive_angle`(它只對視線角速率積分),
所以單改角度參數不會有任何效果。附帶效果正是「俯角隨距離與速度改變」:
視線俯角 `atan(h/d)` 隨接近自然變陡。

⚠️ `thrust` 在 `SET_ATTITUDE_TARGET` 是**直接當百分比**
(`forced_throttle = thrust × 100`),不是對應 `THR_MIN..THR_MAX`。
`THR_MAX=21` ⇒ `trim` 應為 0.105,不是 0.5。

### 5.3 環半徑的地形重算(`tot_node._reground`)

```
pushover_r = (perch_alt − (z_g + dive_alt_agl)) / tan(dive_angle)
```

`z_g` 用**實測地形**(每機各自回報,取平均),不是寫死的 `ground_alt`。
HOME 在中央山脈 1839 m AMSL、紅心 −30 m ⇒ `z_g = −1886 m`。
用寫死的 −48 會讓殘餘高度恆為 1784 m 且**與俯角無關**。

---

## 6. 腳本地圖

| 腳本 | 用途 |
|---|---|
| `swarm_sitl_up.sh` | 起 6× SITL + MAVProxy 分流;寫入所有 ArduPilot 參數 |
| `swarm_sitl_down.sh` | 收掉 SITL |
| `swarm_ros2_up.sh` | 起 MAVROS + sar_swarm 節點 |
| `swarm_ros2_down.sh` | 收掉 ROS 2 層(SITL 不動) |
| `jet_catapult_launch.py` | 彈射起飛 + 爬升到巡航高度 |
| `jet_show_ros2.py` | 驅動長機沿航線前進(ROS 2 編隊模式) |
| `jet_show_cruise.py` | 非 ROS 2 的編隊巡航(V / CONE);備用 |
| `stot_trial.sh` | 一趟完整任務(起飛 → 巡航 → STOT) |
| `stot_repeat.sh` | 連跑 N 趟,量成功率 |
| `stot_score.py` | 驗收計分(觸地基準) |
| `stot_extract_df.py` | dataflash → 全程 CSV |
| `stot_mission_anim.py` / `stot_make_anim.sh` | 出動畫 |
| `jet_design.py` / `jet_verify.py` | 由目標速度反推機體參數 / 驗證 |

### 6.1 執行方式

```bash
source stot_fast.env
CRUISE_MODE=ros2 SHAPE=V PREFIX=X TRIALS=1 bash stot_repeat.sh
```

`stot_fast.env` 是**唯一的參數來源**。⚠️ 一個變數只准在該檔出現一次 ——
重複定義時**後面的勝**,是本專案反覆踩到的坑。

---

## 7. 機體模型

實測兩點反解(`jet_verify.py` 量平飛頂速與 20° 俯衝終端,換算到同一密度):

```
T_eff = W·sinγ / ((V_D/V_L)² − 1)      ← 與阻力無關,只看比值
c_eff = T_eff / V_L²
```

| 量 | 值 |
|---|---|
| 重量 W | 215.8 N |
| 有效阻力常數 c | 0.003168(D = cV²) |
| 推力 | T = 4.142 × `THR_MAX` |
| `THR_MAX` | **21** ⇒ T = 87.0 N,T/W = 0.403 |
| 平飛頂速 | 165.7 EAS / 193 地速 |
| 最佳爬升速度 | √(T/3c) = 95.7 EAS,RoC 25.7 m/s |
| 27° 終端 | 242.7 總速 / **215.3 水平** |

⚠️ **空速 vs 地速**:3100 m 的 σ = 0.7345 ⇒ 地速 = 空速 × **1.167**。
速度**指令**吃空速(`DO_CHANGE_SPEED`、`nominal_speed`),
而間距、追趕距離、驗收的「地速」都是地面量。混用會系統性差 17%。

---

## 8. 這個系統最容易出錯的地方

按踩過的次數排序:

1. **參數沒隨機體改速度重新標定**(5 次)—— 僚機速度、`catch_speed`、
   `weave` 週期、`TECS_SINK_MAX`、`TRIM_THROTTLE` 都留著泡棉機時代的值
2. **變數沒有往下轉發**(多次)—— env 定義了,中間某層沒傳,跑的是預設值
3. **同一變數重複定義**,後面的勝
4. **兩個控制器控同一個量**
5. **分流被先連者獨佔**
6. **空速/地速混用**

**共同特徵:全部靜默。** 沒有錯誤訊息,只會看到「飛機就是做不到」。
看到做不到,先查上面六項,再去懷疑導引律。
