# quadplane_vtol — 4+1 QuadPlane 的 Gazebo 模型

給 `gz sim` 8.14.0 (Harmonic) + ArduPilot SITL (JSON 後端) 用的 QuadPlane 機體。
**本機從零建的** —— `~/ardupilot_gazebo/models/` 底下 11 個目錄裡帶 `ArduPilotPlugin`
的只有 iris / iris_with_gimbal / zephyr / zephyr_with_parachute 四個,**沒有任何
QuadPlane 或 VTOL 模型可抄**。旋翼幾何與轉向抄 iris、機翼與舵面結構抄 zephyr,
其餘全部自己推導。

---

## 1. 邊界:這裡有什麼、沒有什麼

| 檔 | 內容 |
|---|---|
| `model.sdf` | **完整可飛的單機模型**:幾何 + 9 個 lift-drag 面板組 + 9 個 ApplyJointForce + ArduPilotPlugin (含 9 個 `<control>`) |
| `model.config` | 只在 `<include>` 路徑才會被讀。群機路徑不讀它 |
| `README.md` | 本檔。數字來源分級、限制、讀回驗證、群機用法 |

**不在這裡** (別人負責):world 檔、SITL 啟動腳本、`.parm` 參數檔、coordinator。
對他們的硬性要求集中在 §7 與 §8。

**尺寸來源**是 `core/base/airframe.py` 的 `HERO_PNP`。
⚠️⚠️ 那份紀錄裡**沒有任何氣動幾何** —— 沒有翼展、翼面積、弦長、慣量。
所以本模型只有質量與速度是「來自紀錄」,幾何與係數全部是推導或假設。§3 逐項分級。

為什麼選 `HERO_PNP` 而不是 `PIONEER_PNP`:兩台差 3.7 倍質量 (6.65 vs 24.6 kg),
不能共用一個模型。挑 HERO 是因為 (a) 它離可借用的零件尺度近得多
(iris 全機 1.75 kg / zephyr 1.85 kg),(b) 它的 `airframe.py` 註解**明寫了型錄失速值
10 m/s**,讓 §3 的「保守值偏誤」可以量化;PIONEER 只有規劃值 14,沒有型錄值可對照。
要 PIONEER 就得另開一個模型目錄,不要改這個。

---

## 2. 通道對照

端到端鏈:`SERVO(N+1)` → `servos[N]` → `pkt.pwm[N]` → SDF 的 `<control channel="N">`

| ch | 功能 | 關節 | type | servo_min/max | multiplier | offset |
|---|---|---|---|---|---|---|
| 0 | 左副翼 | `aileron_left_joint` | POSITION | 1100 / 1900 | **-0.698** | -0.5 |
| 0 | 右副翼 | `aileron_right_joint` | POSITION | 1100 / 1900 | **+0.698** | -0.5 |
| 1 | 升降舵 | `elevator_joint` | POSITION | 1100 / 1900 | +0.872 | -0.5 |
| 2 | 前推馬達 | `pusher_joint` | VELOCITY | **1000 / 2000** | +900 | 0 |
| 3 | 方向舵 | `rudder_joint` | POSITION | 1100 / 1900 | +0.872 | -0.5 |
| 4 | M1 右前 CCW | `rotor_0_joint` | VELOCITY | **1000 / 2000** | +500 | 0 |
| 5 | M2 左後 CCW | `rotor_1_joint` | VELOCITY | **1000 / 2000** | +500 | 0 |
| 6 | M3 左前 CW | `rotor_2_joint` | VELOCITY | **1000 / 2000** | -500 | 0 |
| 7 | M4 右後 CW | `rotor_3_joint` | VELOCITY | **1000 / 2000** | -500 | 0 |

出處 (本機逐檔看過):

* `ch 0/1/3 = aileron/elevator/rudder`
  → `~/ardupilot/libraries/SITL/SIM_Plane.cpp:407-409`
* `ch 4-7 = QuadPlane 升力馬達`:`SIM_QuadPlane` 的 `motor_offset` 預設 4
  (*leave first 4 servos free for plane*) ⇒ SERVO5-8;JSON 後端下 `SIM_QuadPlane`
  不被實例化,真正生效的是 `SERVOn_FUNCTION`,而 ArduPlane 在 `quadplane.cpp`
  的 runtime 預設把 motor1..4 指到 CH_5..CH_8
* channel 上限 16 → `ArduPilotPlugin.cc:1552`
  `int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;`
* 舵面 `servo_min/max` 1100/1900 = ArduPlane 出廠預設
  → `~/ardupilot/libraries/SRV_Channel/SRV_Channel.cpp:37` (MIN 1100)、`:46` (MAX 1900)、`:55` (TRIM 1500)
* 油門 `servo_min/max` 1000/2000:本 repo 的 `sitl/default_params/plane.parm:25-26`
  **明確覆寫**了 `SERVO3_MIN 1000` / `SERVO3_MAX 2000`

### ⚠️⚠️ 升力馬達不可以抄 iris 的 1100/1900

iris 四個 `<control>` 都寫 `servo_min 1100` / `servo_max 1900`,而
`Tools/autotest/default_params/quadplane.parm` 與本 repo 的
`mission/sitl_launcher.py:281-282` (`generate_vtol_params()`) 都是
`Q_M_PWM_MIN/MAX = 1000/2000`。照抄的話,`ArduPilotPlugin.cc:1677` 的

```cpp
double raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min);
raw_cmd = gz::math::clamp(raw_cmd, 0.0, 1.0);
```

會把 1000–1100 全夾成 0、1900–2000 全夾成 1 ⇒ **四顆一起損失 25% 行程,完全靜默**。

`ArduPilotPlugin.cc:137/142` 的欄位註解自己就寫了
*"The lower/upper bound of PWM input should match SERVOX_MIN/MAX for this channel"*。

⚠️ 另注意 `Q_M_PWM_TYPE` 預設 NORMAL ⇒ 對馬達輸出生效的是 `Q_M_PWM_MIN/MAX`,
`SERVOn_MIN/MAX` **不生效**。所以 `vtol_default.parm` 裡 SERVO5/6 寫 1000/2000
而 SERVO7/8 寫 1100/1900 的不一致對這條路徑沒影響 —— 但也代表「檢查 SERVO5_MIN」
是**錯的檢查**,要檢查 `Q_M_PWM_MIN/MAX`。

### ⚠️ 兩塊副翼共用 `channel="0"` 是合法的

`ArduPilotPlugin.cc:552-575` 的 `LoadControlChannels` 只是
`while (controlSDF) { Control control; ... push_back }`,**沒有任何 channel 唯一性
檢查**。同一段還有一個靜默坑:少寫 `channel` 屬性時 (`:569-573`) 它會**用解析順序
當 channel** 並只發一行 `gzwarn` ⇒ 通道整個錯位卻不失敗。本檔每個 `<control>`
都明寫 `channel`。

---

## 3. 每個數字的來源:三級分類

密度一律 ρ = 1.225 kg/m³ (題目規定的推導用值),重力 g = 9.80665,
W = 6.65 × 9.80665 = **65.21 N**。

⚠️ 這與 iris/zephyr 的 `air_density` 1.2041 不同,是刻意的:模型要跟推導用同一個 ρ,
才能一路對得起來。差異造成失速速度 10.00 → 10.07 m/s (+0.7%)。

### A 級:來自紀錄 (`core/base/airframe.py` 的 `HERO_PNP`)

| 量 | 值 | 欄位 |
|---|---|---|
| 全機質量 | 6.65 kg | `mass_kg` |
| 酬載 | 0.6 kg | `payload_kg` |
| 巡航空速 | 19.0 m/s | `cruise_airspeed_mps` |
| 規劃最低空速 | 16.0 m/s | `min_mission_airspeed_mps` |
| 保守失速空速 | 12.0 m/s | `stall_airspeed_mps` |
| **型錄失速空速** | **10.0 m/s** | 該檔對 `stall_airspeed_mps` 的註解:「型錄失速 10 m/s 是特定測試條件,規劃一律用 12 當保守值」 |
| 轉換空速 | 14.0 m/s | `transition_airspeed_mps` |
| 坡度上限 | 25° | `max_bank_angle_deg` |
| 爬升航跡角 | 3° | `max_climb_path_angle_deg` |
| 設計轉彎半徑 | 100 m | `design_turn_radius_m` |

### B 級:推導 (從 A 級 + C 級假設算出來的)

| 量 | 值 | 推導 |
|---|---|---|
| **翼面積 S** | **0.890 m²** | `S = 2mg/(ρ·V_stall²·CL_max) = 130.43/(1.225·10.0²·1.2) = 0.8873` → 取 0.890 |
| 平均弦長 c | 0.318 m | `S/b = 0.890/2.80` |
| 展弦比 AR | 8.81 | `b²/S = 7.84/0.890` |
| 翼負荷 | 7.47 kg/m² | `6.65/0.890` |
| 3D 升力線斜率 `cla` | 4.95 /rad | `2π/(1 + 2π/(π·e·AR))`,e=0.85 → 4.958 |
| 巡航 CL | 0.331 | `2W/(ρ·19²·S) = 130.43/393.6` |
| 翼型零升角 `a0` | 0.067 rad | `CL_cruise/cla = 0.331/4.95` ⇒ 巡航時機身姿態剛好 0° |
| 失速攻角 `alpha_stall` | 0.242 rad | `CL_max/cla = 1.2/4.95` |
| 水平尾面積 | 0.14 m² | `V_H·S·c/l_h = 0.5·0.890·0.318/1.00` |
| 水平尾 `cla` | 4.20 /rad | AR_h = 0.80²/0.14 = 4.57, e=0.9 → 4.227 |
| 垂直尾面積 | 0.10 m² | `V_V·S·b/l_v = 0.04·0.890·2.80/1.00` |
| 垂直尾 `cla` | 3.70 /rad | 幾何 AR 1.6,端板效應取有效 AR 3.2, e=0.9 → 3.708 |
| 中性點 | x = −0.063 m | `(S_w·cla_w·x_acw + S_h·cla_h·η·x_ach)/(...)`,η=0.9 |
| 靜安定裕度 | 19.7 % MAC | `(0 − (−0.063))/0.318` |
| 巡航阻力 | 6.52 N | `W/(L/D) = 65.21/10.0` |
| 寄生阻力 `CD0` | 0.022 | 由 L/D=10 扣掉誘導阻力 |
| 誘導 `cda` (機翼面板) | 0.166 | `CDi/α = 0.0111/0.067` |
| 升力槳葉面積 | 0.00408 m² | 由 T/W=2.5 反解:每片 20.39 N ÷ (0.5·1.225·80²·4.25·0.30) |
| 升力槳等效弦長 | 21.5 mm | `0.00408/0.19` —— 對 15" 槳是**物理合理值** (真槳 25–30 mm) |
| 前推槳葉面積 | 0.00137 m² | 由「19 m/s 巡航 65% 油門抵銷 6.52 N」反解 |
| 前推螺距 `a0` | 0.45 rad | 由零推力空速 51 m/s 反推 (`105.8·tan(0.45)`) |
| 機體慣量 | 0.747 / 0.713 / 1.227 | Raymer 無因次迴轉半徑 Rx=0.25、Ry=0.38、Rz=0.39 |
| 副翼 `rad_to_cl` | −0.70 | cla·τ·展比,再對「面板 cp 在副翼內側」做力臂補償,最後對半以壓住滾轉率 (見 §5 預測 #2) |
| 升降舵 `rad_to_cl` | −2.73 | `cla_h·τ = 4.20·0.65` (弦比 0.40) |
| 方向舵 `rad_to_cl` | +1.96 | `cla_v·τ = 3.70·0.53` (弦比 0.28) |

#### ⚠️⚠️ 失速值的選擇會系統性地把翼做小

`S ∝ 1/V_stall²`。用保守值 12.0 會得到 **0.616 m²**,比用型錄值 10.0 的
0.887 m² **小 31%**。

`airframe.py` 的 `stall_airspeed_mps` 是**加了裕度的規劃值**,它的用途是
「不要貼著失速規劃航線」,不是「這台飛機的空氣動力學」。拿它去反解翼面積等於
把安全裕度誤當成物理量,做出來的機翼會太小 ⇒ 失速速度反而變高 ⇒ 惡性循環。

**本模型用型錄值 10.0 m/s。** 已把偏誤寫進 `model.sdf` 的 `WING_AREA_HALF` 註解。

### C 級:假設 (紀錄裡沒有,我猜的。旁邊是猜錯會怎樣)

| 假設 | 值 | 猜錯的後果 |
|---|---|---|
| 翼展 / 展弦比 | 2.80 m / AR 8.81 | AR 直接決定誘導阻力與 `cla`。AR 偏大 ⇒ 高估航程、低估滾轉慣量 |
| 機身長 / 尾臂 | 1.80 m / 1.00 m | 決定 `iyy`/`izz` 與兩個尾容積 ⇒ 俯仰與偏航安定度、舵效全部跟著動 |
| Oswald 效率 e | 0.85 (翼) / 0.9 (尾) | 只影響 `cla` 的二階,影響小 |
| `CL_max` | 1.2 | 題目規定。真值偏高 ⇒ 我們的翼偏大 ⇒ 失速速度比真機低 |
| 整機 L/D | 10.0 | **偏樂觀**。四顆外掛旋翼的 VTOL 實務常在 8–11。高估 ⇒ 續航與最大平飛速度被高估 |
| 升力槳半徑 / ω_max / 質量 | 0.19 m (15") / 500 rad/s / 0.05 kg | 只要三者的組合湊出同一個 T/W,飛行差異很小;但陀螺效應與旋翼慣性只跟 ω、I 有關 |
| 前推槳半徑 / ω_max / 質量 | 0.14 m (11") / 900 rad/s / 0.03 kg | 決定零推力空速。ω 偏低 ⇒ 高速段推力提早歸零 ⇒ 飛不到 AIRSPEED_MAX |
| 推重比 T/W | 2.5 | 偏低 ⇒ 懸停沒有姿態控制餘裕;偏高 ⇒ 高度迴路過度靈敏 |
| 旋翼位置 | x ±0.70 / y ±0.55 / z +0.10 | 決定滾轉與俯仰力矩臂 ⇒ 多旋翼段的姿態頻寬 |
| 上反角 | 3° | 沒有它橫向是中性的;太大會出現荷蘭滾 |
| 舵面弦比 / 展比 / 襟翼效率 τ | 0.25/0.40/0.28,τ 0.50/0.65/0.53 | 直接是舵效。錯了 ⇒ ArduPlane 的 `RLL2SRV`/`PTCH2SRV` 調校值失去意義 |
| 舵面偏度上限 | ±20° / ±25° / ±25° | 撞上限會讓控制器在大擾動時飽和 |
| 各 link 質量分配 | base 6.10 / imu 0.15 / 槳 0.05×4 / 前推 0.03 / 舵面 0.05+0.05+0.04+0.03 | 總和被讀回鎖在 6.65 |
| 起落架長度 | 0.20 m | 只影響 spawn 高度與觸地 |
| 關節阻尼 | 旋翼 0.0008 / 前推 0.0003 / 舵面 0.05 | **旋翼那個就是偏航權限**,見 §5 |

---

## 4. 為什麼這樣選

### 4.1 座標慣例:選 iris 的 `0 0 0 180 0 0`

一台機體**只能有一個** `modelXYZToAirplaneXForwardZDown`。
iris 用 `180 0 0`、zephyr 用 `180 0 -90`。選 iris 的:

* zephyr 那個 `-90` 存在,是因為它的機體檔本身是 x=左 / y=後 建的
  (`zephyr_with_ardupilot/model.sdf` 註解:*"Zephyr is oriented x-left, y-back, z-up"*)。
  那是為了遷就既有 mesh 的歷史包袱。
* 我們沒有 mesh 包袱 (§4.3),可以直接用航空慣例 **x = 機頭前方 / y = 左 / z = 上**。
  好處是 SDF 裡每個 `pose` 的 x 就是「離機頭多遠」,不用在腦裡轉 90°。
* **後果**:所有從 zephyr 抄來的 lift-drag `<forward>`/`<upward>` 都必須重算。
  zephyr 機翼的 `forward` 是 `0 -1 0` (它的 −y = 前),我們是 `1 0 0`。
  混用零件時最容易錯的就是這裡,所以 `model.sdf` 每一塊面板都逐塊寫了理由。
* 不變式:本檔**所有 link 的 `<pose>` 旋轉分量都是 0**,所以 `<axis><xyz>` 在
  joint frame / child link frame / model frame 三種解讀下數值相同 ⇒ SDF 版本
  改了 `expressed_in` 預設值也咬不到我們。**之後若有人給 link 加旋轉,這條保護就沒了。**

### 4.2 lift-drag:用普通版,不用 advanced 版

本機兩支都在 (`/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/`, 8.14.0):
`libgz-sim8-lift-drag-system.so` 與 `libgz-sim8-advanced-lift-drag-system.so`。

選普通版,三個理由:

1. advanced 版要一整組穩定導數。對它的 `.so` 跑 `strings` 看得到它讀
   `Cellb / Cellp / Cellq / Cellr / Cem0 / Cema / Cemq / Cemr / Cenb / Cenp /
   Cenq / Cenr / alpha_stall / area / mac / ...` 這一大票。`HERO_PNP` 一個都沒有。
   **憑空捏 20 個導數,比誠實用 6 個面板參數更糟。**
2. 普通版是 iris 與 zephyr 用的那一支 —— 本機唯一有「跟 `ArduPilotPlugin` 在
   `lock_step` 下一起跑過」證據的那一支。
3. 我們需要 `<control_joint_name>` / `<control_joint_rad_to_cl>` 驅動舵面。
   普通版有 (`strings` 在 `.so` 裡找得到 `control_joint_name`,以及被截斷的
   `LiftDragcontrol_joint_ra`)。

⚠️⚠️ **方法教訓 1**:本機**沒有** `LiftDrag.cc` 原始碼 (只有 `.so`,`/usr/src` 沒有),
所以 `cla` / `cda` / `alpha_stall` 的**確切語意我無法從原始碼證實**,是從 iris/zephyr
的數值反推的。反推的內容與可證偽預測見 §5。

⚠️⚠️ **方法教訓 2**:對 `.so` 跑 `strings` 找不到某個 tag **不能當成它不支援**。
短字面值 (≤15 char) 會被 SSO 內嵌進指令流,`strings` 看不到 —— 我實測 `cla_stall`
只以 `cla_stalH` 的殘影出現。用 `strings` 證明「沒有」是無效的證明。

### 4.3 自足式:不 include 任何模型、不參照任何 mesh

全部用 primitive box / cylinder。理由:

* `model://ground_plane` 在本機不存在 ⇒ gz 會去 Fuel 線上抓,離線吐 Error Code 14。
  參照 iris 的 `.dae` 會把 `iris_with_standoffs` 變成硬相依,踩到同一類問題。
* 群機用法是把本檔內文**展開**進 world 檔,那時 mesh 的相對 uri 解析又是另一個坑。
* **最大好處**:`<imuName>` 與 `<jointName>` 不需要模型名前綴。
  對照 `iris_with_ardupilot` 寫的是 `iris_with_standoffs::imu_link::imu_sensor`,
  那個前綴純粹是因為它 `<include>` 了底層模型。我們寫 `imu_link::imu_sensor`,
  展開進 world 並改名成 `vtol_3` 時**不用跟著改**。
  (外掛用 `model.Entity()` 當根去找:`ArduPilotPlugin.cc:619`;找不到印
  *"This plugin will not run."* `:624`。)

代價:看起來很陽春。要好看請另做一個 `*_visual` 版本,不要動這個。

### 4.4 有做方向舵 (ch 3),沒有省略

省掉不會報錯,只會讓 ch3 沒人接。ArduPlane 的協調轉彎、側風落地、QuadPlane 前飛段
的偏航配平全都會送 ch3;沒人接的話飛機還是飛,但**偏航靠 rudder 的那條路徑整條
靜默失效** —— 正是本專案最痛的那類失效 (指令送得出去、沒有告警、就是沒效果)。所以做。

### 4.5 舵面正負號的推導

權威來源是 ArduPilot 自己的參考被控體 `~/ardupilot/libraries/SITL/SIM_Plane.cpp`
(它能被 ArduPlane 飛好,就代表它的號誌約定就是控制器心裡想的那個):

```
:334  la = qbar*b*( ... + c_l_deltaa*inputAileron  + c_l_deltar*inputRudder);
:335  ma = qbar*c*( ... + c_m_deltae*inputElevator);
:336  na = qbar*b*( ... + c_n_deltaa*inputAileron  + c_n_deltar*inputRudder);
```

而 `SIM_Plane.h:75 c_l_deltaa = 0.25`、`:80 c_m_deltae = 1.0`、`:86 c_n_deltar = 0.1`
三個都是**正**的,`la/ma/na` 是 FRD 下的 roll/pitch/yaw 力矩 ⇒

> **正升降舵 = 抬頭 / 正副翼 = 右滾 / 正方向舵 = 右偏航**

(`filtered_servo_angle` 把 PWM 以 `SERVOn_TRIM` 為中心映到 [−1,1];`SERVOn_REVERSED=0`
時 PWM 高於 trim 就是正。)

由此推出本模型的號誌 (完整幾何推導寫在 `model.sdf` 各 `<control>` 的註解):

| 舵 | 軸 | PWM 高於 trim 時 | 面板 `rad_to_cl` |
|---|---|---|---|
| 升降舵 | `0 1 0` | 關節角 + ⇒ 後緣上揚 ⇒ 尾翼下壓 ⇒ 抬頭 | −2.73 |
| 左副翼 | `0 1 0` | 關節角 − ⇒ 後緣下垂 ⇒ 左翼升力增 | −0.70 |
| 右副翼 | `0 1 0` | 關節角 + ⇒ 後緣上揚 ⇒ 右翼升力減 | −0.70 |
| 方向舵 | `0 0 1` | 關節角 + ⇒ 後緣往右 ⇒ 側力朝左 ⇒ 機頭右偏 | +1.96 |

⚠️ **推導對 ≠ 裝對。** 這是全檔最可能裝反的一項,而裝反的症狀是「起飛就翻」
不是報錯。§6.5 有不必起飛的檯面測試。

---

## 5. 已知限制 + 可證偽的預測

### 5.1 限制 (會怎麼影響飛行表現)

| 限制 | 影響 |
|---|---|
| **`gz-sim-lift-drag-system` 沒有 CD0 項** (`cd = \|cda·α\|`)。不補的話阻力隨速度**下降**,飛機俯衝會無上界加速 | 已用一塊 `cla=0` 的 parasite 面板補上 (`PARA_CD0=0.022`)。⚠️ 但 `cla=0` 的行為**沒有原始碼可查**,萬一被當成無效面板整塊跳過,寄生阻力會**靜默消失**而畫面完全看不出來。**這是第一順位要驗的東西**,見 §6.4。回退是把 `cla` 改成 `0.001` |
| 機身 / 旋翼樑 / 起落架 / 停轉旋翼的阻力**沒有各自建模**,全部塞進那一塊 parasite 面板 | 轉換段 (旋翼停轉 + 機身大攻角) 的阻力一定不準;姿態相關的阻力變化只有那塊面板的 `\|cda·α\|` 這一項 |
| `air_density` 寫死 1.225,**不隨高度變** | 3000 m 的升力與推力與海平面完全相同 ⇒ **任何跟高度有關的性能驗收都不成立** |
| 面板的 CL、CD 都對 α 線性 ⇒ 每塊面板的 L/D 是常數 `cla/cda` | 失速後行為只有 `cla_stall`/`cda_stall` 兩個折線段,不是真的分離流 |
| `alpha_stall` 是否包含 `a0`,是**從 zephyr 的數值反推**的 (a0=0.13、alpha_stall=0.339、cla=3.7 ⇒ 含 a0 得 CL_max 1.25 合理,不含得 1.74 太高) | 若反推錯了,本模型的實際 CL_max 會是 1.53 而不是 1.2 ⇒ 失速速度低 12% |
| 沒有電池 / 耗電模型 | **續航完全不可驗**。`catalogue_endurance_min` 在這裡沒有對應物 |
| 沒有旋翼下洗對機翼的干擾、沒有地面效應、沒有旋翼間氣動干擾 | 低空懸停與轉換段的真實度最低 |
| 前推槳等效弦長 9.8 mm (真槳約 18 mm) | 這是外掛用固定 `a0` 取代真實槳葉扭轉所吃掉的差,是**校正黑數**,不要拿它當真實槳葉幾何 |
| 慣量是 Raymer 經驗式估算 | **這台機體調出來的 `Q_A_RAT_*` / `RLL2SRV_*` 不可以外推到真機** |
| 旋翼偏航權限刻意做成物理值 (等效力臂 0.022 m),比 iris 的等效值小約 30 倍 | 若偏航明顯遲鈍 (QLOITER 對不準航向),調大 `rotor_*_joint` 的 `<damping>`;iris 等效值是 0.045 |

### 5.2 可證偽的預測 (第一趟飛就該對照)

這些是本模型設計時算出來的,不是量出來的。**對不上就代表某一項沒生效。**

| # | 預測 | 對不上代表 |
|---|---|---|
| 1 | 懸停時 `raw_cmd ≈ 0.632`,即四顆馬達 PWM ≈ **1632**;`Q_M_THST_HOVER` 應收斂到 **0.63** 附近 | 落在 0.34 附近 ⇒ 升力槳的 `area` 或 `multiplier` 有一項沒生效 |
| 2 | 滿副翼穩態滾轉率 ≈ **77 °/s** (19 m/s) | 遠大於 ⇒ `AIL_RAD_TO_CL` 太大;遠小於 ⇒ 面板 cp 位置或 τ 估錯 |
| 3 | 19 m/s 平飛的升降舵殘餘偏角 ≈ **−1.5°** | 遠大於 ⇒ CG / 機翼 cp / 尾臂 三者有一個錯 |
| 4 | 19 m/s 平飛油門 ≈ **65 %** | 遠低於 ⇒ **寄生阻力面板沒生效** (見 §6.4) |
| 5 | 最大平飛速度 ≈ **31 m/s** | 飛得比這快很多 ⇒ 同上 |
| 6 | 失速速度 ≈ **10.1 m/s** | |
| 7 | 靜態前推推重比 ≈ **0.55** (35.9 N) | |
| 8 | 前推零推力空速 ≈ **51 m/s** | |
| 9 | 旋翼穩態轉速 ≈ **497 rad/s** (P 控制靜差 0.6 %,推力低 1.2 %) | 明顯更低 ⇒ `cmd_max` 不夠 (iris 就是這個病,見下) |

#### ⚠️⚠️ `cmd_max` 才是真正的推力上限,不是 `multiplier`

iris 就踩在這上面:它 `multiplier` 838,但穩態需要的關節力矩是
阻尼 `0.004×838 = 3.35` + 葉片阻力 `0.10` ≈ **3.45 N·m**,而它的 `cmd_max` 只有 **2.5**
⇒ 實際轉速卡在約 612 rad/s,最大推力只有標稱的 **53 %**。
也就是 **iris 的 `multiplier` 從來沒有被達到過**。

我們的:阻尼 `0.0008×500 = 0.40` + 葉片阻力力矩 `0.512` = **0.912 N·m**,
`cmd_max = 4.0` 有 4.4 倍餘裕 ⇒ 標稱轉速真的到得了。讀回有把這條鎖起來 (§6.2)。

---

## 6. 讀回驗證

本專案的硬性要求是「設定值必須讀回驗證,不接受靜默失敗」。SDF 是靜態 XML,
`gz` **不會**展開環境變數,所以「可覆寫 + 讀回」在這一層是這樣做的:

* 每個可調常數都在自己那一行,帶 `@tunable NAME=value` 註解 ⇒ 產生器 / `sed`
  可以確定性改寫,`grep` 得到、`diff` 得出來。
* **讀回不是看註解,是解析真正被寫進 XML 的值**,再與註解交叉比對。
  改註解沒改值、改值沒改註解,兩邊都會被抓。

### 6.1 語法驗證

```bash
export SDF_PATH=/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/sitl/gazebo/models:$HOME/ardupilot_gazebo/models
gz sdf -k model.sdf
```

實跑輸出:

```
Warning [Utils.cc:132] [.../model.sdf:L299]: XML Element[gz_frame_id], child of
element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
Valid.
```

* 那個 warning 是**預期的**:對 `~/ardupilot_gazebo/models/iris_with_standoffs/model.sdf`
  跑同一條指令會得到一模一樣的 warning,而且同樣印 `Valid.`。
* `SDF_PATH` 在這裡其實不需要 (本模型沒有 `<include>`) —— 但驗有 include 的**世界檔**時
  關鍵環境變數就是 `SDF_PATH`,不是 `GZ_SIM_RESOURCE_PATH`。寫在這裡免得下次忘記。

**外掛沒有被靜默丟掉**的檢查 (`-p` 印的是完全解析後的 SDF):

```bash
gz sdf -p model.sdf 2>/dev/null | python3 -c "
import sys, collections, xml.etree.ElementTree as ET
r = ET.parse(sys.stdin).getroot().find('model')
c = collections.Counter(p.get('filename') for p in r.findall('plugin'))
for k, v in sorted(c.items()): print(f'{v:3d} x {k}')
print('links', len(r.findall('link')), 'joints', len(r.findall('joint')))
ap = [p for p in r.findall('plugin') if p.get('name') == 'ArduPilotPlugin'][0]
print('controls', len(ap.findall('control')))"
```

實跑輸出 (2026-09-09):

```
  1 x ArduPilotPlugin
  9 x gz-sim-apply-joint-force-system
  1 x gz-sim-joint-state-publisher-system
 15 x gz-sim-lift-drag-system
links 11 joints 10
controls 9
```

15 塊 lift-drag = 2 (主翼左右) + 1 (水平尾) + 1 (垂直尾) + 1 (寄生阻力)
+ 8 (四顆旋翼各兩片葉) + 2 (前推兩片葉)。9 個 `ApplyJointForce` = 9 個被控關節。
任何一個數字少了,對應的氣動力或舵面就會**完全沒有作用而不報錯**。

⚠️⚠️ **`gz sdf -k` 說 Valid 不代表它是合法的 XML。**
實測:在註解裡塞兩個連續連字號,`gz sdf -k` 照樣印 `Valid.`,而 Python 的 expat
直接 `ParseError: not well-formed (invalid token)`。
(gz 用 TinyXML2,它對 `--` 寬容。) 本檔在開發過程真的踩到一次 —— 原本註解裡寫了
SITL 的命令列旗標,已改寫。⇒ **凡是要餵給嚴格解析器的下游工具 (世界檔產生器、
本節的讀回),都要獨立做一次嚴格解析,不能只信 `gz sdf -k`。**

⚠️⚠️⚠️ **2026-09-09 對抗式複審:同一個坑在 `model.config` 裡活著,而且是紅的。**
它的註解用了一條 52 個連字號的分隔線 ⇒ `gz sdf -k` / gz 載入完全正常,
Python expat 卻是 `ParseError: not well-formed (invalid token): line 6, column 4`。
原因就是上一段自己寫的那件事 —— 但**嚴格解析只對 `model.sdf` 跑過**,
`model.config` 從來沒驗。已改用方框繪製字元 `─`。
⇒ 教訓:嚴格解析要對**這個目錄下的每一個 XML 檔**跑,不是只跑主檔:

```bash
python3 -c "
import glob, xml.etree.ElementTree as ET
for f in sorted(glob.glob('*.sdf') + glob.glob('*.config')):
    ET.parse(f); print(f, 'expat OK')"
```

### 6.2 值層級讀回

把下面這段存成 `readback.py`,`python3 readback.py model.sdf`。結尾 exit code 非 0 = 有硬違規。

```python
#!/usr/bin/env python3
# quadplane_vtol 生效值讀回。只讀檔,不啟動任何模擬。
import re, sys, xml.etree.ElementTree as ET

path = sys.argv[1] if len(sys.argv) > 1 else "model.sdf"
raw = open(path, encoding="utf-8").read()
m = ET.parse(path).getroot().find("model")
bad = []

def f(el, tag, d=None):
    if el is None: return d
    x = el.find(tag); return float(x.text) if x is not None else d
def s(el, tag, d=None):
    if el is None: return d
    x = el.find(tag); return x.text.strip() if x is not None else d
def lock(ok, msg):
    print(("  OK   " if ok else "  FAIL ") + msg)
    if not ok: bad.append(msg)

print("== links ==")
tot = 0.0
linkmap = {}
for L in m.findall("link"):
    linkmap[L.get("name")] = L
    i = L.find("inertial"); mass = f(i, "mass", 0.0); tot += mass
    I = i.find("inertia"); ixx, iyy, izz = (f(I, k) for k in ("ixx", "iyy", "izz"))
    tri = (ixx+iyy >= izz) and (iyy+izz >= ixx) and (izz+ixx >= iyy)
    print(f"  {L.get('name'):16s} m={mass:7.4f}  I=({ixx:.4g},{iyy:.4g},{izz:.4g})"
          f"  triangle={'OK' if tri else 'VIOLATION'}")
    if not tri:
        bad.append(f"link {L.get('name')} 慣量三角不等式不成立 (sdformat Error 19)")
print(f"  TOTAL MASS = {tot:.4f} kg   (HERO_PNP.mass_kg = 6.65)")
if abs(tot - 6.65) > 1e-6:
    bad.append(f"總質量 {tot:.4f} != 6.65 —— 有人改了某個 link 卻沒回頭改 BASE_MASS")

print("== joints ==")
jd = {}
for J in m.findall("joint"):
    a = J.find("axis")
    if a is None: continue
    dyn = a.find("dynamics"); lim = a.find("limit")
    jd[J.get("name")] = (s(a, "xyz"),
                         f(dyn, "damping") if dyn is not None else None,
                         f(lim, "lower") if lim is not None else None,
                         f(lim, "upper") if lim is not None else None)
    print(f"  {J.get('name'):22s} axis={jd[J.get('name')][0]:8s} "
          f"damping={jd[J.get('name')][1]}  limit=[{jd[J.get('name')][2]},{jd[J.get('name')][3]}]")

ap = [p for p in m.findall("plugin") if p.get("name") == "ArduPilotPlugin"][0]
print("== ArduPilotPlugin ==")
print(f"  fdm_addr={s(ap,'fdm_addr')}  fdm_port_in={s(ap,'fdm_port_in')}"
      f"  lock_step={s(ap,'lock_step')}  have_32_channels={s(ap,'have_32_channels')}")
print(f"  imuName={s(ap,'imuName')}")
print(f"  modelXYZToAirplaneXForwardZDown={s(ap,'modelXYZToAirplaneXForwardZDown')}")
print("== controls ==")
ctl, ctl_joints = {}, []
for c in ap.findall("control"):
    ch = int(c.get("channel")); jn = s(c, "jointName")
    rec = dict(ch=ch, joint=jn, type=s(c, "type"), smin=f(c, "servo_min"),
               smax=f(c, "servo_max"), mult=f(c, "multiplier"), off=f(c, "offset"),
               p=f(c, "p_gain"), i=f(c, "i_gain"), d=f(c, "d_gain"),
               force=s(c, "useForce"), cmax=f(c, "cmd_max"), cmin=f(c, "cmd_min"))
    ctl.setdefault(ch, []).append(rec); ctl_joints.append(jn)
    print(f"  ch{ch}  {jn:22s} {rec['type']:8s} servo[{rec['smin']:.0f},{rec['smax']:.0f}]"
          f" mult={rec['mult']:+8.4g} off={rec['off']:+.2f} p={rec['p']}"
          f" useForce={rec['force']} cmd[{rec['cmin']},{rec['cmax']}]")

# lift-drag 面板依 link 分組
LD = [p for p in m.findall("plugin") if p.get("name", "").endswith("LiftDrag")]
bylink = {}
for p in LD: bylink.setdefault(s(p, "link_name"), []).append(p)
def panel(pred, what):
    # 找不到就記成硬違規並回 None —— 絕不讓 AttributeError 取代違規報告。
    # 舊版直接 f(None,...) 會拋 AttributeError,而 6.4 明文寫的回退
    # (寄生面板 cla 0.0 -> 0.001) 正好觸發它 => 照文件操作就讓讀回崩掉。
    for p in LD:
        if pred(p): return p
    bad.append(f"找不到 lift-drag 面板: {what}")
    return None
wing = panel(lambda p: s(p, "control_joint_name") == "aileron_left_joint", "主翼左")
wingR = panel(lambda p: s(p, "control_joint_name") == "aileron_right_joint", "主翼右")
ht = panel(lambda p: s(p, "control_joint_name") == "elevator_joint", "水平尾")
vt = panel(lambda p: s(p, "control_joint_name") == "rudder_joint", "垂直尾")
# 寄生阻力面板用結構認,不是用 cla==0 認:cla 是它唯一被允許調整的欄位。
para = panel(lambda p: s(p, "link_name") == "base_link"
             and p.find("control_joint_name") is None
             and s(p, "cp") == "0 0 0", "寄生阻力(base_link 無舵面、cp 在 CG)")
rot = bylink.get("rotor_0", [None])[0]
pus = bylink.get("pusher", [None])[0]
if rot is None: bad.append("找不到 rotor_0 的葉片面板")
if pus is None: bad.append("找不到 pusher 的葉片面板")

print("== value locks ==")
# 1. PWM 刻度:與 .parm 對齊
for ch in (4, 5, 6, 7):
    r = ctl[ch][0]
    lock(r["smin"] == 1000 and r["smax"] == 2000,
         f"ch{ch} 升力馬達 servo_min/max = {r['smin']:.0f}/{r['smax']:.0f} "
         f"(必須 1000/2000 對齊 Q_M_PWM_MIN/MAX)")
for ch in (0, 1, 3):
    for r in ctl[ch]:
        lock(r["smin"] == 1100 and r["smax"] == 1900,
             f"ch{ch} 舵面 servo_min/max = {r['smin']:.0f}/{r['smax']:.0f} (必須 1100/1900)")
r = ctl[2][0]
lock(r["smin"] == 1000 and r["smax"] == 2000,
     f"ch2 油門 servo_min/max = {r['smin']:.0f}/{r['smax']:.0f} (必須 1000/2000)")

# 2. <type> / <useForce>:兩個「錯了幾乎不出聲」的欄位
#    type 打錯不會讓通道停擺,ArduPilotPlugin.cc:587-596 只 gzwarn 一行然後
#    默默退回 VELOCITY。打在舵面上 => POSITION 變 VELOCITY,舵面不再定位而是
#    以 cmd (rad/s) 漂移直到撞上 <limit> => 起飛就翻,線索只有開機日誌裡那一行。
#    useForce=0 + POSITION => :1401-1404 只印一行 "Failed to do position
#    control on joint ...",那支舵面整趟不動。
WANT_TYPE = {0: "POSITION", 1: "POSITION", 2: "VELOCITY", 3: "POSITION",
             4: "VELOCITY", 5: "VELOCITY", 6: "VELOCITY", 7: "VELOCITY"}
for ch, rs in sorted(ctl.items()):
    for r in rs:
        lock(r["type"] == WANT_TYPE.get(ch),
             f"ch{ch} {r['joint']} type={r['type']} (必須 {WANT_TYPE.get(ch)}; "
             f"打錯會靜默退回 VELOCITY,ArduPilotPlugin.cc:587-596)")
        lock(r["force"] == "1",
             f"ch{ch} {r['joint']} useForce={r['force']} (必須 1; POSITION+useForce=0 "
             f"只印一行 warning 然後整支舵面不動,ArduPilotPlugin.cc:1401-1404)")

# 3. 轉向與對稱
lock(ctl[4][0]["mult"] > 0 and ctl[5][0]["mult"] > 0
     and ctl[6][0]["mult"] < 0 and ctl[7][0]["mult"] < 0,
     "quad-X 轉向 ch4/5 需為正 (CCW)、ch6/7 需為負 (CW)")
lock(ctl[0][0]["mult"] * ctl[0][1]["mult"] < 0, "ch0 兩塊副翼 multiplier 必須反號")

# 4. 四顆旋翼必須一模一樣
#    這一段是補洞:同一個常數在 XML 裡寫了四份,讀回只看 rotor_0 的話
#    「只改其中一顆」是全綠的。四顆不對稱的症狀是起飛就偏,沒有任何錯誤訊息。
CHJ = {4: "rotor_0", 5: "rotor_1", 6: "rotor_2", 7: "rotor_3"}
for ch, rl in CHJ.items():
    lock(ctl[ch][0]["joint"] == rl + "_joint",
         f"ch{ch} 必須接 {rl}_joint (實際 {ctl[ch][0]['joint']})")
def same(getter, what):
    vals = [getter(k) for k in CHJ]
    lock(len(set(vals)) == 1, f"四顆旋翼 {what} 不一致: {vals}")
same(lambda ch: f(linkmap[CHJ[ch]].find("inertial"), "mass"), "質量")
same(lambda ch: tuple(f(linkmap[CHJ[ch]].find("inertial/inertia"), k)
                      for k in ("ixx", "iyy", "izz")), "慣量")
same(lambda ch: jd[CHJ[ch] + "_joint"][1], "damping (= 偏航權限)")
same(lambda ch: jd[CHJ[ch] + "_joint"][0], "轉軸")
same(lambda ch: abs(ctl[ch][0]["mult"]), "|multiplier|")
same(lambda ch: (ctl[ch][0]["p"], ctl[ch][0]["i"], ctl[ch][0]["d"]), "PID 增益")
same(lambda ch: (ctl[ch][0]["cmin"], ctl[ch][0]["cmax"]), "cmd_min/max")
same(lambda ch: (ctl[ch][0]["smin"], ctl[ch][0]["smax"]), "servo_min/max")
same(lambda ch: tuple(sorted((f(p, "area"), f(p, "a0"), f(p, "cla"), f(p, "cda"),
                              abs(float(s(p, "cp").split()[0])))
                             for p in bylink.get(CHJ[ch], []))),
     "葉片面板 (面積/a0/cla/cda/cp)")

# 5. 面板與外掛數量:少一塊就是那一段氣動力靜靜消失
#    刪掉一塊葉片面板 => 那顆旋翼推力剩一半,而 gz sdf -k 照樣 Valid.、
#    讀回照樣印得出 rotor_0 的數字。所以逐 link 數。
WANT_PANELS = {"base_link": 5, "rotor_0": 2, "rotor_1": 2, "rotor_2": 2,
               "rotor_3": 2, "pusher": 2}
lock({k: len(v) for k, v in bylink.items()} == WANT_PANELS,
     f"lift-drag 面板逐 link 數量 = {sorted((k, len(v)) for k, v in bylink.items())} "
     f"(必須 {sorted(WANT_PANELS.items())})")
lock(len(LD) == 15, f"lift-drag 面板總數 {len(LD)} (必須 15)")
lock(len(m.findall("link")) == 11 and len(m.findall("joint")) == 10,
     f"links={len(m.findall('link'))} joints={len(m.findall('joint'))} (必須 11 / 10)")
ajf = {s(p, "joint_name") for p in m.findall("plugin")
       if p.get("name", "").endswith("ApplyJointForce")}
miss = set(ctl_joints) - ajf
lock(not miss, f"缺少 ApplyJointForce 的被控關節: {sorted(miss)}")
lock(not (set(ctl_joints) - set(jd)),
     f"jointName 指到不存在的關節: {sorted(set(ctl_joints) - set(jd))}")

# 6. 座標框:degrees 屬性掉了 = 180 被當弧度 = 姿態全錯,而且不報錯
for tag in ("modelXYZToAirplaneXForwardZDown", "gazeboXYZToNED"):
    e = ap.find(tag)
    lock(e is not None and e.get("degrees") == "true",
         f"{tag} 缺 degrees=true (180 會被當成弧度)")
imu_sensor = linkmap["imu_link"].find("sensor") if "imu_link" in linkmap else None
ipose = imu_sensor.find("pose") if imu_sensor is not None else None
lock(ipose is not None and ipose.get("degrees") == "true",
     "imu_sensor 的 pose 缺 degrees=true")
lock(s(ap, "imuName") == "imu_link::imu_sensor"
     and imu_sensor is not None and imu_sensor.get("name") == "imu_sensor",
     f"imuName={s(ap,'imuName')} 必須是 imu_link::imu_sensor 且該 sensor 真的存在 "
     f"(ArduPilotPlugin.cc:1122-1123 以 model 為根解析,自足式模型不加模型名前綴)")

# 7. 推力上限:cmd_max 才是真上限,不是 multiplier
#    穩態力矩 = 關節阻尼力矩 + 兩片葉的氣動阻力力矩,兩項都從解析到的值算,
#    不可以寫死:寫死的話改 ROTOR_BLADE_AREA / ROTOR_CP / ROTOR_WMAX 這條鎖不會動。
#    阻力係數用 gazebo lift-drag 的式子 cd = |cda x alpha|,alpha 在懸停 = a0
#    (葉片速度純切向)。前一版把 cd 寫成 cda (漏掉 x alpha) => 高估 1/a0 = 3.33 倍。
RHO = 1.225
def blade_torque(p, omega):
    cp = [abs(float(v)) for v in s(p, "cp").split()]
    r = max(cp)
    q = 0.5 * RHO * (omega * r) ** 2
    return q * f(p, "area") * abs(f(p, "cda") * f(p, "a0")) * r
w4 = abs(ctl[4][0]["mult"])
need = abs(jd["rotor_0_joint"][1]) * w4 + sum(blade_torque(p, w4) for p in bylink["rotor_0"])
lock(ctl[4][0]["cmax"] > need * 1.5,
     f"升力馬達 cmd_max={ctl[4][0]['cmax']} 需 > 1.5x 穩態力矩 {need:.3f} N.m")
wp = abs(ctl[2][0]["mult"])
needp = abs(jd["pusher_joint"][1]) * wp + sum(blade_torque(p, wp) for p in bylink["pusher"])
lock(ctl[2][0]["cmax"] > needp * 1.5,
     f"前推 cmd_max={ctl[2][0]['cmax']} 需 > 1.5x 穩態力矩 {needp:.3f} N.m")

# 8. 寄生阻力面板 (6.4 的第一順位風險)
#    讀回只能證明「值寫對了」,不能證明外掛真的算了阻力 —— 那要飛,見 6.4。
#    但至少鎖住:cla 只允許 0.0 (預設) 或 0.001 (6.4 明文回退)。
lock(para is not None and f(para, "cla") in (0.0, 0.001),
     f"寄生阻力面板 cla={f(para,'cla')} 只允許 0.0 或 6.4 的回退值 0.001")

port = int(s(ap, "fdm_port_in"))
lock((port - 9002) % 10 == 0 and port >= 9002,
     f"fdm_port_in={port} 必須是 9002+10*instance (SITL_cmdline.cpp:258-259,444-449)")
print(f"  info  fdm_port_in {port} => SITL 必須用 -I{(port - 9002) // 10}")

# 左右主翼面板必須對稱 (只鎖左邊的話,改右邊是全綠的)
for k, g in (("面積", "area"), ("cla", "cla"), ("a0", "a0"), ("cda", "cda"),
             ("alpha_stall", "alpha_stall"), ("副翼增益", "control_joint_rad_to_cl")):
    lock(f(wing, g) == f(wingR, g),
         f"左右主翼面板 {k} 不一致: {f(wing,g)} vs {f(wingR,g)}")

def link(n): return linkmap[n]
expect = {
    "BASE_MASS": f(link("base_link").find("inertial"), "mass"),
    "ROTOR_MASS": f(link("rotor_0").find("inertial"), "mass"),
    "ROTOR_R": float(link("rotor_0").find("collision/geometry/cylinder/radius").text),
    "ROTOR_CP": abs(float(s(rot, "cp").split()[0])),
    "ROTOR_DAMPING": jd["rotor_0_joint"][1],
    "ROTOR_WMAX": abs(ctl[4][0]["mult"]),
    "ROTOR_PGAIN": ctl[4][0]["p"],
    "ROTOR_BLADE_AREA": f(rot, "area"),
    "PUSHER_MASS": f(link("pusher").find("inertial"), "mass"),
    "PUSHER_R": float(link("pusher").find("collision/geometry/cylinder/radius").text),
    "PUSHER_CP": abs(float(s(pus, "cp").split()[1])),
    "PUSHER_DAMPING": jd["pusher_joint"][1],
    "PUSHER_WMAX": ctl[2][0]["mult"],
    "PUSHER_BLADE_AREA": f(pus, "area"),
    "PUSHER_A0": f(pus, "a0"),
    "WING_AREA_HALF": f(wing, "area"), "WING_CLA": f(wing, "cla"),
    "WING_A0": f(wing, "a0"), "WING_ALPHA_STALL": f(wing, "alpha_stall"),
    "WING_CDA": f(wing, "cda"),
    "AIL_RAD_TO_CL": f(wing, "control_joint_rad_to_cl"),
    "HTAIL_AREA": f(ht, "area"), "HTAIL_CLA": f(ht, "cla"),
    "ELEV_RAD_TO_CL": f(ht, "control_joint_rad_to_cl"),
    "VFIN_AREA": f(vt, "area"), "VFIN_CLA": f(vt, "cla"),
    "RUD_RAD_TO_CL": f(vt, "control_joint_rad_to_cl"),
    "PARA_CD0": round(f(para, "cda", 0.0) * f(para, "a0", 0.0), 6),
    "AIL_LIMIT": jd["aileron_left_joint"][3],
    "ELEV_LIMIT": jd["elevator_joint"][3],
    "RUD_LIMIT": jd["rudder_joint"][3],
    "FDM_PORT_IN": float(port), "LOCK_STEP": float(s(ap, "lock_step")),
}
print("== @tunable marker vs parsed value ==")
pairs = re.findall(r"@tunable\s+([A-Z0-9_]+)\s*=\s*([-\d.eE+]+)", raw)
marks = dict(pairs)
# dict() 會把同名標記默默壓成一個 => 先擋掉重複,否則「只改其中一處」是全綠的
lock(len(pairs) == len(marks),
     f"有重複的 @tunable 名稱 ({len(pairs)} 個標記 / {len(marks)} 個名字)")
for k, v in sorted(marks.items()):
    if k not in expect:
        print(f"  ??   {k:18s} 標記 {v} —— 讀回表沒有這一項,補進 expect{{}}")
        bad.append(f"@tunable {k} 沒有讀回對照"); continue
    ok = expect[k] is not None and abs(float(v) - expect[k]) < 1e-9
    print(f"  {'OK  ' if ok else 'FAIL'} {k:18s} marker={v:<10s} parsed={expect[k]}")
    if not ok: bad.append(f"@tunable {k}: 註解寫 {v}, XML 實際 {expect[k]}")
for k in expect:
    if k not in marks: print(f"  ??   {k:18s} 有讀回但沒有 @tunable 標記")

print("\n== RESULT ==")
if bad:
    for b in bad: print("  VIOLATION:", b)
    sys.exit(1)
print(f"  0 violations. total mass {tot:.4f} kg, {len(ctl_joints)} controls, "
      f"{len(marks)} tunables all match.")
```

實跑結果 (2026-09-09,未突變的 `model.sdf`):

```
  TOTAL MASS = 6.6500 kg   (HERO_PNP.mass_kg = 6.65)
  ch0  aileron_left_joint     POSITION servo[1100,1900] mult=  -0.698 off=-0.50 p=10.0 cmd[-2.5,2.5]
  ch0  aileron_right_joint    POSITION servo[1100,1900] mult=  +0.698 off=-0.50 p=10.0 cmd[-2.5,2.5]
  ch1  elevator_joint         POSITION servo[1100,1900] mult=  +0.872 off=-0.50 p=10.0 cmd[-2.5,2.5]
  ch2  pusher_joint           VELOCITY servo[1000,2000] mult=    +900 off=+0.00 p=0.3  cmd[0.0,4.0]
  ch3  rudder_joint           POSITION servo[1100,1900] mult=  +0.872 off=-0.50 p=10.0 cmd[-2.5,2.5]
  ch4  rotor_0_joint          VELOCITY servo[1000,2000] mult=    +500 off=+0.00 p=0.3  cmd[-4.0,4.0]
  ch5  rotor_1_joint          VELOCITY servo[1000,2000] mult=    +500 off=+0.00 p=0.3  cmd[-4.0,4.0]
  ch6  rotor_2_joint          VELOCITY servo[1000,2000] mult=    -500 off=+0.00 p=0.3  cmd[-4.0,4.0]
  ch7  rotor_3_joint          VELOCITY servo[1000,2000] mult=    -500 off=+0.00 p=0.3  cmd[-4.0,4.0]
  info  fdm_port_in 9102 => SITL 必須用 -I10

== RESULT ==
  0 violations. total mass 6.6500 kg, 9 controls, 33 tunables all match.
```

### 6.3 突變測試:證明讀回真的會咬

⚠️ 依專案記憶 *mutation-scripts-must-not-race*:突變**絕不就地改寫生產檔**,
每個突變寫到自己的暫存副本再驗。

⚠️⚠️⚠️ **2026-09-09 對抗式複審:原本那 13 臂全部都咬,但它們咬的是同一類東西。**
另一個 agent 拿五個「這個模型真正會壞掉的方式」去打原版讀回,**四個是全綠的**:

| 打法 | 真實後果 | 原版讀回 |
|---|---|---|
| 舵面 `<type>` 從 `POSITION` 打錯 | `ArduPilotPlugin.cc:587-596` **默默退回 VELOCITY** ⇒ 舵面不再定位,以 rad/s 漂到 `<limit>` ⇒ 起飛就翻 | **綠** |
| 舵面 `<useForce>` 改成 0 | `:1401-1404` 只印一行 warning,那支舵面**整趟不動** | **綠** |
| 只改 `rotor_2` 的 `<damping>` | 四顆反扭矩不對稱 ⇒ 起飛就偏 | **綠**(讀回只看 `rotor_0`) |
| 刪掉 `rotor_3` 的一塊葉片面板 | 那顆推力剩一半 ⇒ 起飛就翻 | **綠** |
| 照 §6.4 把寄生面板 `cla` 改成 0.001 | (這是文件自己寫的回退) | **AttributeError 崩掉**,不是違規報告 |

前四項全是本專案「**檢查器放在不可能為假的地方**」那一類:同一個常數在 XML 裡
寫了四份而讀回只看一份、或那個欄位根本沒被讀。已補進 §6.2(鎖數從 15 條增到
**95 條**),下表是補完之後的實跑結果 —— **20/20 照預期**,含兩個必須維持綠色的
對照臂:

| 臂 | 突變 | 結果 |
|---|---|---|
| A1 | 升力馬達 `servo_min` 抄成 iris 的 1100 | **咬** `ch4 升力馬達 servo_min/max = 1100/2000` |
| A2 | ch6 由 CW 翻成 CCW | **咬** `quad-X 轉向 ch4/5 需為正、ch6/7 需為負` |
| A3 | 拿掉方向舵的 `ApplyJointForce` | **咬** `缺少 ApplyJointForce 的被控關節: ['rudder_joint']` |
| A4 | `base_link` 質量 6.10 → 6.20 | **咬** `總質量 6.7500 != 6.65` |
| A5 | 只改 `@tunable` 註解不改值 | **咬** `ROTOR_WMAX: 註解寫 520, XML 實際 500.0` |
| A6 | 只改值不改 `@tunable` 註解 | **咬** `ROTOR_WMAX: 註解寫 500, XML 實際 520.0` |
| A7 | `fdm_port_in` 改成 irlock 的 9105 | **咬** `必須是 9002+10*instance` |
| A8 | 兩塊副翼同號 | **咬** `ch0 兩塊副翼 multiplier 必須反號` |
| A9 | 四顆 `cmd_max` 砍到 **0.8** | **咬** `cmd_max=0.8 需 > 1.5x 穩態力矩 0.554 N.m` |
| A10 | 舵面 `servo_max` 抄成 2000 | **咬** `ch1 舵面 servo_min/max = 1100/2000` |
| A11 | 旋翼慣量破壞三角不等式 | **咬** 四個 link 全報 `慣量三角不等式不成立`;`gz sdf -k` 同時報 `Error Code 19: A link named rotor_1 has invalid inertia.` |
| A13 | 註解裡塞兩個連續連字號 | **咬** (ParseError);⚠️ 同一個檔 `gz sdf -k` 印 **`Valid.`** |
| A12 | 對照組:只改註解文字不動值 | **綠** (exit 0) —— 證明它不是無條件變紅 |
| B1 | `rotor_0` 的 `<type>` 打成 `VELOCTIY` | **咬** `ch4 rotor_0_joint type=VELOCTIY (必須 VELOCITY)` |
| B2 | 升降舵 `<type>` `POSITION`→`VELOCITY` | **咬** `ch1 elevator_joint type=VELOCITY (必須 POSITION)` |
| B3 | 升降舵 `<useForce>` 1→0 | **咬** `ch1 elevator_joint useForce=0 (必須 1)` |
| B4 | 只改 `rotor_2` 的 `<damping>` (x100) | **咬** `四顆旋翼 damping 不一致: [0.0008, 0.0008, 0.08, 0.0008]` |
| B5 | 只改 `rotor_1` 的質量 | **咬** `四顆旋翼 質量 不一致: [0.05, 0.06, 0.05, 0.05]` |
| B6 | 刪掉 `rotor_3` 的一塊葉片面板 | **咬** 3 條(葉片面板不一致 + 逐 link 面板數 + 總數 15) |
| B8 | 寄生面板 `cla` 0.0→0.5 (當升力面用) | **咬** `寄生阻力面板 cla=0.5 只允許 0.0 或 0.001` |
| B9 | 拿掉 `modelXYZToAirplaneXForwardZDown` 的 `degrees="true"` | **咬** `缺 degrees=true (180 會被當成弧度)` |
| B10 | 只改**右**主翼面板的 `cla` | **咬** `左右主翼面板 cla 不一致: 4.95 vs 4.5` |
| B11 | 同一個 `@tunable` 名字出現兩次 | **咬** `有重複的 @tunable 名稱 (34 個標記 / 33 個名字)`(原版 `dict()` 會默默壓成一個) |
| B12 | 只改 `rotor_0` 的 `cmd_max` | **咬** `四顆旋翼 cmd_min/max 不一致` |
| B13 | `imuName` 加上模型名前綴 | **咬** `必須是 imu_link::imu_sensor` |
| B14 | 前推 `cmd_max` 4.0→0.3 | **咬** `前推 cmd_max=0.3 需 > 1.5x 穩態力矩 0.369 N.m` |
| B7 | 對照組:照 §6.4 把寄生面板 `cla` 改成 0.001 | **綠** (exit 0) —— 文件寫的回退操作不可以讓讀回崩掉 |

A12 / B7 這兩臂是必要的:沒有它們,「其他 18 個突變都變紅」也可能是因為讀回
**恆為紅**。

⚠️⚠️ 順帶訂正一個數字:A9 的門檻從 0.912 變成 **0.554 N.m**,因為原本的
「葉片阻力力矩 0.512」把 gazebo lift-drag 的阻力係數寫成 `cda`,而它其實是
`|cda x alpha|`(懸停時 `alpha = a0 = 0.3`)⇒ 高估 `1/a0 = 3.33` 倍。
新版讀回**不再寫死這個常數**,而是從解析到的 `area / cp / cda / a0 / multiplier`
現算,所以改葉片幾何時這條鎖會跟著動。連帶訂正 `model.sdf` 的 `ROTOR_DAMPING`
註解:每顆反扭矩 0.91 → **0.554 N.m**、等效力臂 0.022 → **0.0136 m**。

### 6.4 第一順位:確認寄生阻力面板真的生效

`cla=0` 的面板行為本機沒有 `.cc` 可查(`/usr/src` 底下沒有、apt 只裝 `.so`),
失效方式是**靜默消失**。

⭐ 2026-09-09 對抗式複審補了一條**唯讀**證據,把這個風險降了一級:
對 `libgz-sim8-lift-drag-system.so` 做 `strings` 抽出**全部**人看得懂的訊息,
它只有這幾條護欄——

```
The LiftDrag system requires the 'link_name' parameter
Link with name[ ] not found. The LiftDrag will not generate forces
Multiple link entities with name[ ] found. Using the first one.
Joint with name[ ] not found.
The LiftDrag system should be attached to a model entity.
```

**沒有任何一條與 `cla` / `area` / 係數有效性有關** ⇒ 外掛不會因為 `cla=0` 就整塊
跳過;它只驗 `link_name` / `control_joint_name` 找不找得到。
⚠️ 但這**只證明它不會被丟掉**,不證明 `cd` 真的被算進去(`cd` 的分支在
`PreUpdate` 裡,沒有字串可抽)。下面的飛行判準還是要做:

1. 進 FBWA / CRUISE,配平在 **19 m/s** 平飛,記油門 → 應該 ≈ **65 %**
2. 加速到 **28–30 m/s** 平飛,記油門 → 應該接近 **100 %**,而且再也加不上去

如果第 2 步在 60 % 油門就飛到 35 m/s 以上,寄生阻力沒生效。
回退:把那塊面板的 `<cla>0.0</cla>` 改成 `<cla>0.001</cla>` 再驗一次
(升力貢獻 1e-4 級,可忽略)。

### 6.5 舵面號誌:不用起飛的檯面測試

機體停在地上,SITL 進 **MANUAL**,逐支動:

```bash
# 讀關節角 (world 名字換成實際的)
gz topic -e -t /world/<WORLD>/model/<MODEL>/joint_state -n 1
```

| 動作 | 應該看到 |
|---|---|
| 俯仰桿**往後** (抬頭) | `elevator_joint` 角度 **> 0** (後緣上揚) |
| 滾轉桿**往右** | `aileron_left_joint` **< 0** 且 `aileron_right_joint` **> 0** |
| 方向舵桿**往右** | `rudder_joint` 角度 **> 0** (後緣往右) |

任何一支反了,只要把該 `<control>` 的 `<multiplier>` 變號,**同時**把對應面板的
`<control_joint_rad_to_cl>` 變號 —— 兩個都改才等於「舵面往同一邊、氣動力也跟著」。
只改一個會變成「舵面看起來對、力反了」,那是最難查的一種。

### 6.6 每一架都活著:`sim_time` 前進**不充分**

`lock_step` 的失效語意 (`ArduPilotPlugin.cc`):

* 已經上線過的機體斷線 ⇒ `arduPilotOnline` **永遠不會被設回 false**
  (`:1523-1548` 的 lock_step 分支走 `SendState()` 重送,不是 `arduPilotOnline = false`)
  ⇒ `PreUpdate` 卡在 `:1204-1213` 的 `while (!ReceiveServoPacket() && arduPilotOnline)`,
  每輪睡 10 ms。gz-sim 的 `PreUpdate` 跑在**單一更新執行緒** ⇒
  **一架死掉會凍結整個 world**。而且是 sleep 不是 busy-spin ⇒ **沒有 CPU 尖峰,
  用 CPU 判別會失效**。
* 從來沒收到第一個封包的機體 ⇒ `arduPilotOnline` 仍是 false ⇒ 迴圈立刻退出 ⇒
  **world 照跑,那架靜靜死掉**。

⇒ 判準:`/stats` 的 `sim_time` 前進是**必要但不充分**條件。要證明每一架都活著,
必須**逐機**:每架 SITL 的 log 有 `JSON received` 且**沒有**持續的
`No JSON sensor message`;或逐機讀 MAVLink 的姿態/位置確實在變。

### 6.7 埠:`ss -lunp` 證明不了任何事

`ArduPilotPlugin.cc:227` 是 `SocketUDP sock = SocketUDP(true, true);`,
`SocketUDP.cc:74` 設 `SO_REUSEADDR` ⇒ 兩個 socket bind 同一個 `127.0.0.1:port`
**都會成功**,而且**後 bind 的收走全部封包、先 bind 的收到零**,
沒有錯誤、沒有 log。⇒ 前置檢查必須**實際送探測封包並確認誰收到**,不能只看 `ss`。

---

## 7. 群機用法:內嵌展開,不要 `<include>`

已被證實可行的多機寫法 (本機四機 iris 世界檔實測 `sim_time` 前進、RTF 0.236) 是:
**把整個 model 的內文 (含它自己那份 `ArduPilotPlugin`) 內嵌展開進同一個 world 檔**,
只 `<include>` 最底層的裸模型。本模型是自足式的 (§4.3),**沒有底層裸模型可 include**,
所以整段 `<model>…</model>` 直接複製 N 份即可。

每一架**必須改**的只有三處:

| 位置 | 改成 |
|---|---|
| `<model name="quadplane_vtol">` | 唯一名字,例如 `vtol_10`、`vtol_11` |
| `<pose>0 0 0.25 0 0 0</pose>` | 各機起降點;**z 必須 ≥ 0.25**(起落架最低點在 base_link 座標 z = −0.25),給 0 會在第一個 tick 吃到巨大接觸力 |
| `<fdm_port_in>9102</fdm_port_in>` | `9002 + 10 × instance` |

⚠️⚠️⚠️ `<pose>` 那一行是**就地改**,不可以「保留原本那行、在後面再補一個」。
2026-09-09 實測:同一個 `<model>` 底下放兩個 `<pose>`,`gz sdf -p` **兩個都原樣
印出來、零 warning**,而 sdformat 的 `Element::Get` 取的是**第一個**
⇒ 產生器附加在後面的座標被靜靜丟掉,**N 架全部疊在 `0 0 0.25` 同一點**。
沒有錯誤訊息,畫面上只看到「怎麼只有一架」。

**不用改**:`<imuName>` 與所有 `<jointName>`。它們是相對於外掛所在 model 解析的
(`ArduPilotPlugin.cc:619`),改 model 名字不影響。

⭐ 2026-09-09 把「N 份複本會不會互相搶 link/joint」這件事查到底(唯讀,`nm -D`):

| 外掛 | 它用的查名 API | 作用域 |
|---|---|---|
| `ArduPilotPlugin` (imu) | `entitiesFromScopedName(imuName, ecm, model.Entity())` `:1122-1123` | **以自己的 model 為根** |
| `ArduPilotPlugin` (joint) | `JointByName(ecm, model.Entity(), jointName)` `:619` | **以自己的 model 為根** |
| `LiftDrag` | `gz::sim::v8::entitiesFromScopedName(...)` (`nm -D` 抽到的未定義符號) | **以自己的 model 為根** |
| `ApplyJointForce` | `gz::sim::v8::Model::JointByName(...)` | **以自己的 model 為根** |

⇒ N 份複本各自有 `base_link` / `rotor_0_joint` / `imu_link::imu_sensor` 是安全的,
不會發生「15 x N 塊 lift-drag 全部把力施到第一架身上」。這是「內嵌展開 N 份」
這條路走得通的**機制證據**,不只是「四機 iris 跑起來了」的軼事。
⚠️ 如果有人改成用 `<include>` 包一層,這些名字就**必須**補上模型名前綴,
否則外掛整個不跑,只印一行 `This plugin will not run.` (`:624`)。

### 埠算術 (⚠️ 與 instance 綁死,不能各算各的)

ArduPilot 的 JSON 後端自己從 instance 號推:

```
~/ardupilot/libraries/AP_HAL_SITL/SITL_cmdline.cpp:258   const int SIM_IN_PORT  = 9003;
                                                  :259   const int SIM_OUT_PORT = 9002;
                                                  :444-449  等於預設值時 += instance*10
```

SITL **送**伺服封包到 `SIM_OUT_PORT` ⇒ `fdm_port_in == 9002 + 10 × instance`。

| instance | `fdm_port_in` | irlock (UDP) |
|---|---|---|
| 10 | 9102 | 9105 |
| 11 | 9112 | 9115 |
| 12 | 9122 | 9125 |
| … | … | … |

本線從 **-I10** 起,因為 I0–I3 的 9002/9012/9022/9032 已被另一個 session 的
四機 Gazebo 佔用。錯位的症狀是 `No JSON sensor message`,跟撞名一模一樣、分不出來。
`readback.py` 有把「`fdm_port_in` 必須是 `9002+10i`」鎖起來並印出對應的 `-I` 值。

### 其他世界檔層級的注意事項 (不是本模型的檔,但會害死本模型)

* `GZ_PARTITION` 一定要設 (例如 `vtolswarm`),與別人的 gz 隔離。
* `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH` 只寫在 `.bashrc` ⇒
  非互動 shell 讀不到,而 gz 的錯誤訊息不會指向這個原因。腳本要明確 `export`。
* 世界檔必須載入 `gz-sim-imu-system`,否則本模型的 IMU 不產生資料,而
  `ArduPilotPlugin` 只會安靜地送不出狀態。
  (`scripts/wsl/gazebo/taoyuan_runway.sdf` 已經有。)
* SITL 的 `--home` 必須與該 world 的 `<spherical_coordinates>` 原點一致
  (目前 `25.0777 / 121.2325 / 32`,桃園),否則座標基準會差開。
* ⚠️ 桃園世界檔有個未解問題:地面在世界 z = 0,而原點 `elevation` 寫 32 m
  ⇒ **Gazebo 的「海平面」其實是 32 m AMSL**。這次先照 32 做。

---

## 8. 給 `.parm` 擁有者的對齊清單

本模型的刻度與轉向已經定了,參數檔必須配合。以下每一項不對都是**靜默**失效:

| # | 參數 | 必須 / 建議 | 理由 |
|---|---|---|---|
| 1 | `Q_M_PWM_MIN` / `Q_M_PWM_MAX` | **必須 1000 / 2000** | 本模型 ch4–7 的 `servo_min/max` 就是 1000/2000。不一致 ⇒ 行程被夾掉,靜默 |
| 2 | `SERVO1/2/4_MIN` / `_MAX` | **必須維持 1100 / 1900** (即不要覆寫) | 舵面 `<control>` 照 `SRV_Channel.cpp:37,46` 的出廠預設寫 |
| 3 | `SERVO3_MIN` / `_MAX` | **必須 1000 / 2000** | `plane.parm:25-26` 已經是,別動 |
| 4 | `Q_FRAME_CLASS` / `Q_FRAME_TYPE` | **1 / 1** (Quad / X) | 本模型的旋翼佈局與轉向就是 quad-X |
| 5 | `Q_M_THST_EXPO` | **建議 1.0** | AP 的推力線性化模型是 `thrust = expo·act² + (1−expo)·act`;本模型的 sim 推力**恰好是** `act²` ⇒ `expo = 1.0` 才是精確匹配。留 0.65 會讓 AP 對推力的預期系統性偏高 |
| 6 | `Q_M_THST_HOVER` | **建議起始 0.63** (開 `Q_M_HOVER_LEARN`) | 由 T/W = 2.5 推:懸停 `raw_cmd = sqrt(1/2.5) = 0.632`。留 0.34 會讓初期高度迴路嚴重欠推 |
| 7 | `Q_M_SPIN_MAX` | 0.95 可用,但知道後果 | 上限 PWM 1950 ⇒ 實際可用 T/W = 2.26 而不是 2.50 |
| 8 | `AIRSPEED_CRUISE` / `_MIN` / `_MAX` | **建議 19–20 / 14 / 30** | ⚠️ `sitl/default_params/vtol_default.parm` 目前是 **24 / 20 / 34**,而本模型是照 `HERO_PNP` 巡航 19、失速 10.1、最大平飛約 31 設計的。24 要 79 % 油門,34 **飛不到**;`AIRSPEED_MIN 20` 也遠高於 `HERO_PNP.min_mission_airspeed_mps = 16` |
| 9 | `Q_ASSIST_SPEED` | 16–18 | `HERO_PNP.transition_airspeed_mps = 14`,留 2–4 m/s 裕度 |
| 10 | 參數底座 | 用 `~/ardupilot/Tools/autotest/default_params/quadplane.parm` 起頭 | 那是上游調過的完整 QuadPlane 基準。⚠️ **不要**從 `mission/sitl_launcher.py` 的手寫 `vtol_default.parm` 起頭;⚠️ `Tools/autotest/default_params/plane.parm` **不存在**,不要寫進指令 |

⚠️ 第 8 項是**跨檔不一致**,不是我這邊能單方面解決的。要嘛改 `.parm` 配合本模型,
要嘛改本模型配合 `.parm` (改 `WING_AREA_HALF` 等於重做 §3 的整條推導)。
兩邊改哪一邊都可以,但**不能兩邊都不動就開飛** —— 那會得到「飛得起來但每個
KPI 都差一截」而且找不到原因的那種航次。

---

## 9. 已經驗過什麼 / 沒驗過什麼

### 驗過 (本階段只寫檔,不啟動 gz sim 或 SITL)

* `gz sdf -k model.sdf` → **`Valid.`** (exit 0),唯一 warning 與 iris 相同
* 嚴格 XML 解析 (Python expat) → 通過
* `readback.py` → **0 violations**(**95 條鎖**),總質量 6.6500 kg、9 個 control、
  33 個 `@tunable` 全對上;從 README 抽出來直接跑過(確認貼上的那份就是驗過的那份)
* 突變測試 → **20/20 照預期**,含**兩個**必須維持綠色的對照臂
* 三個檔都 **0 個 CR**(CRLF 會讓下游 WSL 腳本整層起不來,而錯誤訊息指向錯的行)
* `model.sdf` **與 `model.config`** 都通過 Python expat 嚴格解析
  (⚠️ `model.config` 原本是紅的,見 §6.1)
* 所有引用的原始碼行號都在本機 `grep`/`sed` 確認過
* `libgz-sim8-lift-drag-system.so` 的護欄清單(`strings`)、四個外掛的查名作用域
  (`nm -D`)、`<pose>` 重複時 sdformat 取第一個(`gz sdf -p` 實測)

### 沒驗過 (只有真的飛才知道)

* §5.2 的九個預測,一個都沒量過
* 舵面三支的正負號 (推導自 `SIM_Plane` 的係數,幾何推導寫在 SDF 註解,但沒實測)
* 寄生阻力面板的 `cd` 是否真的被算進去 (**第一順位**,§6.4;
  「不會被整塊跳過」已有唯讀證據,「`cd` 有值」沒有)
* `alpha_stall` 是否包含 `a0` (從 zephyr 反推,沒有原始碼佐證)
* 旋翼偏航權限夠不夠 (刻意做成物理值,比 iris 小約 30 倍)
* 慣量估算是否合理 ⇒ 姿態 PID 是否可調得動
* 多機同 world 時本模型的 RTF (單機 primitive 幾何應該很輕,但沒量過)
