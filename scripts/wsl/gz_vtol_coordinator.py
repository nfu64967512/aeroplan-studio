#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_vtol_coordinator.py — Gazebo(gz sim Harmonic)+ ArduPilot SITL 的**異質機隊** coordinator。

跑在 WSL。連 N 架 SITL(每架一條 TCP 鏈路),做:連線 → 請求並**量測**資料串流 →
逐機參數讀回契約 → 全機解鎖 → VTOL 起飛到指定高度 → 確認到達 → 全機降落。
另有 `goto` 子指令示範 COMMAND_INT / DO_REPOSITION。

    # 建議用法:埠、profile、原點、PWM 刻度全部讀啟動器/產生器寫下的生效值
    python3 gz_vtol_coordinator.py check   --from-state <RUN>/vtol_swarm.env

    python3 gz_vtol_coordinator.py check   --instances 10,11,12
    python3 gz_vtol_coordinator.py watch   --instances 10:vtol,11:vtol,12:copter
    python3 gz_vtol_coordinator.py mission --instances 10,11 --takeoff-alt 30
    python3 gz_vtol_coordinator.py goto    --instances 10 --lat 25.08 --lon 121.235 \
                                           --alt 60 --loiter-radius 80

⚠️ 預設子指令是 `check`(不解鎖、不起飛)。會動螺旋槳的只有 `mission` / `goto`,
   而且都要顯式打出來。理由:這台機器上常常同時有別人的航次在跑。


══════════════════════════════════════════════════════════════════════
一、為什麼是這支程式(它到底在防什麼)
══════════════════════════════════════════════════════════════════════

Gazebo 多機這條線上,**所有已知的失效表徵都是「看起來正常」**:

* ⚠️⚠️ `ss -lunp` 看到「埠都 bind 了」**證明不了任何事**。ArduPilotPlugin.cc:227
  用 `SocketUDP(true, true)`,SocketUDP.cc:72-75 設 SO_REUSEADDR ⇒ 兩個 socket
  bind 同一個 127.0.0.1:port **都會成功**,而**後 bind 的收到全部封包、先 bind 的收到零**,
  無錯誤、無 log(2026-09-06 本機實測)。⇒ 埠衝突在「檢查埠有沒有被佔」這一層是**隱形的**。
* ⚠️⚠️ lock_step 的兩種死法(ArduPilotPlugin.cc:1204-1213 / 1523-1548):
  已上線過的機體斷線 ⇒ `arduPilotOnline` 永遠不會被設回 false ⇒ PreUpdate 卡在
  `while (!ReceiveServoPacket() && arduPilotOnline)` 每輪睡 10 ms;gz-sim 的 PreUpdate
  跑在**單一更新執行緒** ⇒ **一架死掉凍結整個 world**,而且因為是 sleep 不是 busy-spin,
  **CPU 沒有尖峰** ⇒ 用 CPU 判別會失效。
  反過來,從沒收到第一個封包的機體 ⇒ `arduPilotOnline` 仍是 false ⇒ 迴圈立刻退出 ⇒
  **world 照跑,那架靜靜死掉**。
  ⇒ **`/stats` 的 sim_time 有前進是必要條件但不充分。**

所以這支 coordinator 的存在理由是:**它是唯一逐機、從飛控端量到的活著證據**。
它提供三層互不相依的證據(見 §三):

    (a) 逐機收到訊息      ── 鏈路活著
    (b) 逐機**實測**訊息頻率 ── 串流真的開著(不是預設值長得剛好像有)
    (c) 逐機 time_boot_ms 前進速率 ── **那一架自己的模擬時鐘**在走
                                      (= 那一架的 lock_step 沒卡死)

(c) 是關鍵:world 的 sim_time 前進只證明「至少有一架在餵 FDM」。
SITL 的時鐘由 FDM 封包驅動,所以某一架被卡住時,**它自己的** time_boot_ms 會停,
而其他機的照走 ⇒ 逐機比對才分得出來。這正是事實【3】要的那個判準。


══════════════════════════════════════════════════════════════════════
二、選型與設計決定(每一項都附理由)
══════════════════════════════════════════════════════════════════════

### 2.1 pymavlink,不是 MAVSDK

MAVSDK 本機**完全沒裝**;而且它的 ArduPlane 模式映射只有 11 條、**沒有任何 Q* 模式**
(VTOL 用不了),`goto_location_async` 又寫死 `param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE`
—— 本專案已把「編隊層帶 CHANGE_MODE」列為實測災難(見 memory: strike-authority-handoff)。
另外一個 System = 一個 mavsdk_server process,16 架要 16 個。

### 2.2 「抄」`hv_dccpp/mavlink_link.py` 的做法,不 import 它

⚠️ 這**不是**風格偏好,是實測結果:

    $ cd /mnt/c/.../aeroplan-studio && python3 -c 'import hv_dccpp.mavlink_link'
    File ".../hv_dccpp/mission.py", line 34, in <module>
        from .geo import DemGrid
    File ".../hv_dccpp/geo.py", line 23, in <module>
        import shapely
    ModuleNotFoundError: No module named 'shapely'

`mavlink_link.py:47` 是 `from .mission import ...`,而 mission → geo → shapely。
WSL 這一側沒有 shapely(numpy 有,1.21.5)⇒ **import 得不到**,除非把整套規劃器
相依裝進 WSL。而且任務要求不准改那些檔,所以也不能拆它的 import。

⇒ 本檔沿用它**驗證過的做法**並逐條標出處:

    set_params 讀回      mavlink_link.py:1632-1647(PARAM_SET → PARAM_VALUE 帶容差比對)
    get_param            mavlink_link.py:1649-1657
    command_long + ACK   mavlink_link.py:1659-1670
    set_mode 讀回        mavlink_link.py:1672-1687(先看 ACK,再用 HEARTBEAT.custom_mode 確認)
    心跳/target_system   mavlink_link.py:1469-1506(只認非 GCS 心跳,sysid 由 srcSystem 決定)

同一個 repo 裡 `scripts/wsl/contract_verify.py:236-268 from_fc()` 與
`scripts/wsl/preflight_check.py:71-93 read_params()` 也都是自己接 pymavlink,
沒有 import hv_dccpp —— 本檔與它們同一層,做法一致。

### 2.3 輪詢用 `selectors`(不是逐機同步序列)

⚠️ `FleetLink.poll()`(mavlink_link.py:1936-1960)是**完全同步序列**:
`for u, link in self.links.items(): link.read_telemetry(timeout_s)`。
16 架 × 0.5 s ⇒ 最壞一輪 8 s,而且是每一步都付這個代價。

本檔改成 **`selectors` + 共用總預算**(`Fleet.pump` / `Fleet.gather`):

    送:  一次對所有機體送出,不等回應
    收:  一次 `sel.select()` 等所有 fd,只叫醒真的有資料的那幾條
    ⇒ 一輪的成本是**總預算**(如 3 s),不是 N × 逐機 timeout

⚠️ selector 有一個和 EOF 互相放大的陷阱:**已關閉的 TCP socket 永遠是 readable**
⇒ 死掉的鏈路會讓 `select()` 立刻返回、無限空轉。所以 EOF 守衛(§2.4)除了計數,
**還要把死鏈路從 selector 上 unregister**。兩件事要一起做才有用。

### 2.4 EOF 守衛(pymavlink mavtcp 對端關閉後會噴到爆)

pymavlink/mavutil.py:1273-1289 `mavtcp.recv()`:

    if len(data) == 0:
        self.handle_eof()          # :1268-1271 → print("EOF on TCP socket"); self.reconnect()

而 `recv_match(blocking=True, timeout=T)`(:505-533)在拿不到訊息時只是
`self.select(timeout/2)` 再繞回來 —— 而已關閉的 socket **select 立刻返回可讀** ⇒
每一圈都印一行 EOF,直到外層 timeout 到期。本專案實測噴過 **151 MB**。

⚠️ 注意 EAGAIN/EWOULDBLOCK 走的是 :1281-1282 的 `return ""`,**不會**碰到 handle_eof。
   ⇒ `handle_eof` 一旦被呼叫就是**對端真的關了**,一次就已經是結論。
   本檔預設 `--eof-limit 3` 只是留一點餘裕,重點是**停下來並指名是哪一架**,不是忍受它。

做法:把 `conn.handle_eof` **換成綁在 instance 上的計數器**(蓋過 class method,不改 pymavlink),
只在第 1 次與到達上限時各印一行。並且從不呼叫 `recv_match(blocking=True)`。

### 2.5 埠位:⚠️⚠️ **預設算術只在「SITL 沒被指定 --base-port」時成立**

    SERIAL0 的 TCP 埠 = base_port + 0
    base_port         = 5760 + 10 × instance      ← **只有沒給 --base-port 才是這樣**

出處(2026-09-06 逐行查證,只讀不改 —— 見 §2.6):

    libraries/AP_HAL_SITL/SITL_cmdline.cpp:251   const int BASE_PORT = 5760;
    libraries/AP_HAL_SITL/SITL_cmdline.cpp:434-437  case 'I': if (_base_port == BASE_PORT) _base_port += _instance * 10;
    libraries/AP_HAL_SITL/SITL_State.h:29-38     _serial_path[9] = { "tcp:0:wait", "tcp:2", "tcp:3", …, "tcp:5", … }
    libraries/AP_HAL_SITL/UARTDriver.cpp:371     _listen_sockaddr.sin_port = htons(_sitlState->base_port() + port);

⇒ SERIAL0 → base+0、SERIAL1 → base+2、SERIAL2 → base+3(這正好解釋事實【13】實測到的
  5760 / 5762 / 5763 一路到 5790 / 5792 / 5793)。`--serial` 可切。

⚠️⚠️ **但是** SITL_cmdline.cpp:519 的 `--base-port` 會直接覆寫 `_base_port`,
   而 :434-437 的加法有守衛 `if (_base_port == BASE_PORT)` ⇒ 顯式給過 `--base-port`
   之後,`-I` **不再**加 instance×10(除非你給的正好是 5760,那個值會讓守衛誤判為
   「還是預設」而照加 —— 這是同一行程式碼裡的陷阱)。
   ⇒ 只要啟動器用了 `--base-port`,本檔的預設算術就是**錯的**,而錯法是
     「連到別人的飛機」或「連不上」,不會有人提醒你。

   同目錄的 `gz_vtol_up.sh` 正是這樣:它顯式把 SITL 的 TCP 段挪到 `TCP_BASE`(預設 6360)
   起,而且是**以機序 k 編號**(`gz_port_probe.py:159-168` 的 `base = tcp_base + 10*k`),
   **不是**以 instance 編號 —— I0=10 的四架會落在 6360/6370/6380/6390,
   照 `5760+10*instance` 或 `6360+10*instance` 算都會算錯。

⇒ 因此本檔提供三條路,**越後面越硬**:
     ① `--instances 10,11 --tcp-base 6360 --instance-base 10`
        port = tcp_base + stride×(instance − instance_base) + serial 偏移
     ② `--instances 10:vtol@6360,11:vtol@6370`   ← 逐機顯式埠,任何算術都繞過
     ③ `--from-state <RUN>/vtol_swarm.env`       ← **建議用這個**
        直接讀 `gz_vtol_up.sh` §10 寫下的生效值(`VTOL_INSTANCES` / `VTOL_TCP_BASES` /
        `VTOL_PROFILE_<k>`)。`gz_vtol_world.py:1142-1143` 明文寫著
        「啟動器與 coordinator **必須讀這一份**,不要各自再算一次埠」——
        兩邊各算各的正是這條線上最貴的那一類 bug。
        ⚠️ 讀不到、對不齊、機數不一致一律**大聲失敗**,絕不悄悄退回預設算術:
          「拿不到就用預設」正是本專案要消滅的那個形狀。

⚠️ SERIAL0 的預設路徑是 `tcp:0:wait`,`wait` ⇒ UARTDriver.cpp:415-424 **阻塞在 accept()**,
   沒有 GCS 接上就完全不跑 FDM 迴圈,表徵與「收不到 FDM 封包」一模一樣。
   本檔**是** TCP client,連上去這件事本身就會解掉那道閘門 —— 但也代表:
   * 如果啟動腳本用了 `--serial0 tcp:0`(不帶 :wait,SITL_cmdline.cpp:350-351/506/516),
     SITL 不等我們,我們是普通 client。
     ⭐ `gz_vtol_up.sh` **預設就是這個**(`SERIAL0_MODE=nowait`),而且理由值得記:
       它的逐機驗活閘門必須能在 coordinator 接上**之前**跑完,否則「SITL 起不來」與
       「還沒有人連上去」永遠分不開。⇒ 正常情況下本檔連上時 SITL 早就在跑了,
       所以「連上之後很久才有心跳」不該被解釋成「它在等我們」,要往 lock_step 那邊查。
       state 檔的 `VTOL_SERIAL0_MODE` 會告訴你那一趟到底是哪一種(本檔會把它印出來);
   * 如果別的東西(MAVProxy / MP)已經占著同一個埠,SITL 的 listen backlog 只有 1,
     我們會連不上或搶不到。這時用 `--serial 1`(base+2)或 `--serial 2`(base+3)。

⚠️ 事實【13】:另一個 session 的四機 Gazebo 佔著 TCP 5760…5793 與 UDP 9002/9012/9022/9032。
   ⇒ 本線用 `-I10` 起(TCP 6360+)。所以 `--instances` **沒有預設值**,不給就報錯 ——
   給一個「常用預設」等於埋一顆連到別人飛機上的地雷。

### 2.6 不碰 ~/ardupilot、不碰 ~/ardupilot_gazebo

本檔只講 MAVLink,一行原始碼都不改、一次 waf 都不叫。
(那棵樹有另一個 session 未提交的 SIM_JSBSim 改動,而且 build 開著 autoconfigure。)


══════════════════════════════════════════════════════════════════════
三、串流:⚠️⚠️ **同一份程式在 plane 與 copter 上會長不同的臉**
══════════════════════════════════════════════════════════════════════

libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp(2026-09-06 逐行確認):

    :49-59   #elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
             AP_MAV_DEFAULT_STREAM_RATE_POSITION 1     ← 1 Hz
             AP_MAV_DEFAULT_STREAM_RATE_EXTRA1   1     ← 1 Hz
    :60-71   #elif APM_BUILD_COPTER_OR_HELI || …
             …_POSITION 0 / …_EXTRA1 0 / …_EXT_STAT 0  ← **全部 0**

⇒ 同一套埠算術、同一份 coordinator:plane 給你 1 Hz(低到會讓相位/同步表現異常,
  但**不足以讓人懷疑鏈路**),copter 是**全黑**。這是異質機隊最容易吃到的靜默陷阱。

⇒ 本檔**無條件**請求串流,並且**量實際收到的頻率**才算數:

    1. `MAV_CMD_SET_MESSAGE_INTERVAL`(511;GCS_Common.cpp:5891)逐則設定 —— 有 COMMAND_ACK
    2. `MAV_CMD_GET_MESSAGE_INTERVAL`(510;GCS_Common.cpp:5894 / 3350-3376)讀回 MESSAGE_INTERVAL
       (回 -1 = disabled、0 = 這個訊息不存在)
    3. `REQUEST_DATA_STREAM` 當保底(舊韌體/沒有 511 時)—— ⚠️ 它**沒有 ACK**,
       所以它自己不構成任何證據
    4. **實測**:在 `--rate-window` 秒內數收到幾則,算 Hz,低於門檻就報錯並**指名是哪一架**

⚠️ 串流速率參數**已改名**:現在是 `MAV0_` / `MAV1_` / `MAV2_*`
   (ArduPlane/Parameters.cpp:987 `GOBJECT(_gcs, "MAV", GCS)`),
   舊的 `SR0_*` 會被韌體**靜默略過**。本檔一個 `SR*` 都不寫;
   要用參數路徑的人請用 `MAV<n>_` 開頭,並且**照樣要量頻率**。


══════════════════════════════════════════════════════════════════════
四、異質機隊:機型判別與逐機指令差異(全部逐行查證過)
══════════════════════════════════════════════════════════════════════

### 4.1 ⚠️⚠️ **不能用 HEARTBEAT.type 判斷「這台是不是 VTOL」**

    ArduPlane/quadplane.cpp:4042-4048  MAV_TYPE QuadPlane::get_mav_type() const
        if (mav_type.get() == 0) return MAV_TYPE_FIXED_WING;

`Q_MAV_TYPE` 預設 0 ⇒ **QuadPlane 在 MAVLink 上長得和純固定翼一模一樣**。
⇒ 心跳只用來分「plane 韌體 vs copter 韌體」(pymavlink `mode_mapping_bynumber`:
  MAV_TYPE_FIXED_WING / VTOL_* → `mode_mapping_apm`,QUADROTOR 等 → `mode_mapping_acm`)。
  要分 vtol 與純 plane,唯一硬證據是 **`Q_ENABLE` 讀回**。
⇒ 所以 `--instances 10:vtol,11:copter` 的 profile 是**宣告**,而本檔會拿讀回值去**打臉宣告**:
  宣告 vtol 但 Q_ENABLE 讀不到或是 0 ⇒ 判違規並指名。

### 4.2 ⚠️⚠️ 起飛指令:**同一個 world 裡兩種機體要用不同 frame、不同正負號**

    ArduPlane  GCS_MAVLink_Plane.cpp:781-782   只 dispatch MAV_CMD_NAV_TAKEOFF
                                               (**沒有** NAV_VTOL_TAKEOFF ⇒ 送了會 UNSUPPORTED)
               GCS_MAVLink_Plane.cpp:890-897   frame 必須是 MAV_FRAME_LOCAL_OFFSET_NED,
                                               其他一律 MAV_RESULT_DENIED
               GCS_MAVLink_Plane.cpp:894       takeoff_alt = -packet.z   ← **z 是負的**
               GCS_MAVLink_Plane.cpp:859-877   COMMAND_LONG 進來時韌體自己轉:
                                               frame=LOCAL_OFFSET_NED、out.z = -in.param7
               quadplane.cpp:4002-4015         do_user_takeoff 前提:GUIDED + 已解鎖 + 尚未在飛

    ArduCopter GCS_MAVLink_Copter.cpp:488-490  NAV_TAKEOFF **與** NAV_VTOL_TAKEOFF 都收
               GCS_MAVLink_Copter.cpp:580-582  frame 必須是 MAV_FRAME_GLOBAL_RELATIVE_ALT(=3,
                                               **不是** _INT=6),其他 MAV_RESULT_DENIED
               GCS_MAVLink_Copter.cpp:590      takeoff_alt = +packet.z   ← **z 是正的**

⇒ 本檔**預設用 COMMAND_LONG 送起飛**(param7 = 高度,公尺,向上為正),
  刻意讓**各家韌體自己**做 frame 轉換 —— 因為會分岔的正是 frame 與正負號,
  而每家韌體最知道自己那一套(Plane:859-877 / Copter:600-608 `mav_frame_for_command_long`)。
  `--takeoff-command-int` 可切成顯式 COMMAND_INT(本檔自己按 profile 填 frame 與正負號)。
  兩條路徑都**讀 COMMAND_ACK** 並把 MAV_RESULT 的名字印出來。

### 4.3 ⚠️ 降落只能走「切模式 + 讀回」

ArduPlane 的 `handle_command_int_packet`(GCS_MAVLink_Plane.cpp:731-801)裡
**沒有** `MAV_CMD_NAV_VTOL_LAND`、也沒有 `MAV_CMD_NAV_LAND`。
⇒ 送指令是打空氣(會拿到 UNSUPPORTED)。降落一律用 **QLAND(plane)/ LAND(copter)**
  切模式,並用 **HEARTBEAT.custom_mode 讀回**確認,再等 SAFETY_ARMED 清掉。

### 4.4 ⚠️⚠️ DO_REPOSITION 的三個地雷(全部在 GCS_MAVLink_Plane.cpp:575-607)

    :585-586   if ((param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) || control_mode == guided)
    :600-602   if (!isnan(param3) && param3 > 0) mode_guided.set_radius_and_direction(param3, …)
    :606       return MAV_RESULT_FAILED;      ← 上面那個 if 不成立就走這裡

    (a) **param3 絕對不可以是 0**。0 不通過 `param3 > 0` ⇒ active_radius 維持 0 ⇒
        退回 |WP_LOITER_RAD| × eas2tas² ≈ 2030 m ⇒ **飛機會忠實地繞著目標盤旋而不是飛過去**。
        NaN 同樣不通過。⇒ 本檔有硬守衛,param3 ≤ 0 或非有限值一律**拒送**。
    (b) param2 = 0 且**不在 GUIDED** ⇒ 整發走 :606 回 MAV_RESULT_FAILED。
        本專案曾把這個症狀記成「零告警」—— 其實飛控有回 FAILED,**靜默是我們自己不讀 COMMAND_ACK**。
        ⇒ 本檔預設 param2 = 0,但**先讀回確認在 GUIDED** 才送;`--reposition-change-mode` 才帶旗標
        (帶了就是讓飛控替你切模式,那正是被列為實測災難的那個行為)。
    (c) DO_REPOSITION 只在 `handle_command_int_packet`(:734-735)。
        COMMAND_LONG 雖然有 :5319-5335 `command_long_stores_location` + :5365-5380
        `try_command_long_as_command_int` 的轉接,但 COMMAND_LONG 的 lat/lon 是 **float32**,
        裝不下 degE7(25.0777 → 250777000 要 9 位有效數字)⇒ 精度直接掉到公尺級。
        ⇒ 本檔用 **COMMAND_INT**(x/y 是 int32 degE7)。`VehicleLink` 沒有 command_int,
          所以這是本檔自己補的那一塊。


══════════════════════════════════════════════════════════════════════
五、座標基準
══════════════════════════════════════════════════════════════════════

⚠️⚠️ 原點與升力馬達 PWM 刻度的**權威來源是 `<world>.ports.json`**(gz_vtol_world.py 產,
   :1428-1429 明文要求「啟動器與 coordinator 必須讀這一份,不要各自再算一次」)。
   本檔的 `--origin-*` 預設值與舊版寫死的 1000/2000 都只是**沒有對照表時的退路**:
   * `--ports-json <world>.ports.json`(用 `--from-state` 時會由 `VTOL_WORLD` 自動推出來)
     ⇒ 原點取自 `origin.*`;PWM 期望值取自**那一架自己**的 `channels[].servo_min/max`。
   * 沒有對照表 ⇒ `Q_M_PWM_MIN/MAX`(vtol)與 `MOT_PWM_MIN/MAX`(copter)判
     **「未驗證 ⇒ 違規」**,不是判過。理由:那兩個值不是常數 ——
     上游 iris 的 SDF 是 **1100/1900** 而 ArduCopter 預設 1000/2000,
     我們的 VTOL 是 1000/2000 而那還能被 `GZ_VTOL_LIFT_PWM_MIN/MAX` 改掉。
     把常數抄進本檔 ⇒ 在**唯一真的對不上**的情況下亮綠燈(事實【7】的 25% 行程損失)。

⚠️ SITL 的 `--home` 必須與 world 的 `<spherical_coordinates>` 原點一致,否則座標基準會差開。
   `scripts/wsl/gazebo/taoyuan_runway.sdf:50-56` 是 25.0777 / 121.2325 / **elevation 32**。
   ⚠️ 那個檔有一個未解問題:地面在世界 z=0,而 elevation 寫 32 m
      ⇒ Gazebo 的「海平面」其實是 32 m AMSL。本檔照 32 做,並在 `--origin-alt` 留可覆寫。

⚠️ 高度判準一律用 `GLOBAL_POSITION_INT.relative_alt`(相對 HOME),**不用 alt(AMSL)**。
   理由見 memory: relative-frame-kills-datum-problems —— 只量差量,整類 datum 麻煩一起消掉。
   AMSL 只在原點比對那一步用到,而且門檻放寬。


══════════════════════════════════════════════════════════════════════
六、可調常數
══════════════════════════════════════════════════════════════════════

⚠️ 本專案硬性要求:**設定值必須讀回驗證,不接受靜默失敗**。
所以每個常數都:(1) 可用命令列或環境變數覆寫、(2) 啟動時印出**生效值與它的來源**
(cli / env / default)、(3) `--dump-config` 可輸出 JSON 供事後判讀。
格式比照 `scripts/wsl/contract_verify.py` 的「生效值契約」行 —— 事後才問
「那一趟到底跑的是什麼」的時候,不必靠記憶或猜。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import selectors
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Sequence

# ⚠️ ArduPilot 用 MAVLink 2;COMMAND_INT / MESSAGE_INTERVAL 等都要 MAVLink 2 才完整,
#    而且這個環境變數**必須在 import pymavlink 之前**設定才有效
#    (與 hv_dccpp/mavlink_link.py:39 同一個理由與同一個寫法)。
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil as _mavutil
    _mav = _mavutil.mavlink
except ImportError:  # pragma: no cover
    sys.stderr.write(
        "✗ 需要 pymavlink(WSL 已裝 2.4.49):python3 -m pip install --user pymavlink\n")
    raise

#: 生效值表頭的字面值,方便事後在 log 裡 grep 出「那一趟到底跑的是什麼」。
#: ⚠️ 原註解寫「被下游判讀腳本 grep,改了要兩邊一起改」——**目前沒有任何腳本 grep 它**
#:    (2026-09-09 全 repo 搜過,只有本檔自己的定義與使用兩處)。
#:    寫一句不存在的相依是本專案 memory: promised-guards-that-do-not-exist 那一類:
#:    下一個人會相信有東西在鎖它,於是不去補真正的鎖。真的要接下游判讀時,
#:    請同時加一條會咬人的測試,不要只留註解。
CONFIG_SENTINEL = "gz_vtol 生效值"


# ══════════════════════════════════════════════════════════════════════
# 一、可調常數:宣告 → 解析 → **讀回印出**
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class Knob:
    """一個可調常數。`cli` 是 argparse 的 dest 名,`env` 是環境變數名。

    ⚠️ 三者缺一不可:能覆寫、有預設、**印得出來生效值是哪一個來源給的**。
       只有前兩項的話,「我明明設了」與「它其實吃了預設」看起來一模一樣。
    """
    cli: str
    env: str
    default: Any
    cast: Callable[[str], Any]
    why: str


def _csv(s: str) -> list[str]:
    return [x.strip() for x in str(s).split(",") if x.strip()]


#: ⚠️ 布林值**只認**這兩組字面值。原版寫 `... in ("1","true","yes","on")`,
#:   任何打錯的字(`GZ_VTOL_STRICT=ture`、`=Y`、`=enabled`)都會靜靜變成 False ——
#:   而 False 的 STRICT 正好是「失敗不以非 0 結束」,亦即**打錯一個字就把閘門關掉**,
#:   而且關掉的理由與「我故意關的」在輸出上一模一樣。這正是本檔要消滅的形狀。
_TRUE_WORDS = ("1", "true", "yes", "on")
_FALSE_WORDS = ("0", "false", "no", "off")


def _b(s: Any) -> bool:
    if isinstance(s, bool):
        return s
    t = str(s).strip().lower()
    if t in _TRUE_WORDS:
        return True
    if t in _FALSE_WORDS:
        return False
    raise ValueError(
        f"{s!r} 不是布林值;只接受 {'/'.join(_TRUE_WORDS)} 或 {'/'.join(_FALSE_WORDS)}"
        f"(打錯的字原本會靜靜變成 False,而 False 就是把閘門關掉)")


KNOBS: tuple[Knob, ...] = (
    Knob("instances", "GZ_VTOL_INSTANCES", None, str,
         "SITL instance 清單,格式 `10,11,12` 或 `10:vtol,11:vtol,12:copter`。"
         "⚠️ 故意**沒有預設值** —— 事實【13】另一個 session 佔著 5760…5793,"
         "給一個常用預設等於埋一顆連到別人飛機上的地雷"),
    Knob("host", "GZ_VTOL_HOST", "127.0.0.1", str,
         "SITL 所在主機。WSL 內部一律 127.0.0.1"),
    Knob("tcp_base", "GZ_VTOL_TCP_BASE", 5760, int,
         "SITL_cmdline.cpp:251 BASE_PORT。只有 SITL 也用 --base-port 改過才需要動這個"),
    Knob("tcp_stride", "GZ_VTOL_TCP_STRIDE", 10, int,
         "SITL_cmdline.cpp:434-437 `_base_port += _instance * 10`"),
    Knob("instance_base", "GZ_VTOL_INSTANCE_BASE", 0, int,
         "埠位算術的 instance 原點:port = tcp_base + stride×(instance − instance_base)。"
         "⚠️ 預設 0 = 韌體原生算術(5760+10×instance)。"
         "gz_vtol_up.sh 那種『TCP_BASE 對應 I0』的佈局要設成它的 I0(預設 10),"
         "否則會算到別人的埠上;最保險是 --from-state 直接讀它寫下的生效值"),
    Knob("serial", "GZ_VTOL_SERIAL", 0, int,
         "接哪一條 SITL 序列埠。0→base+0、1→base+2、2→base+3"
         "(SITL_State.h:29-38 + UARTDriver.cpp:371)。"
         "⚠️ SERIAL0 被 MAVProxy/MP 佔住時改用 1 或 2"),
    Knob("profile", "GZ_VTOL_PROFILE", "vtol", str,
         "沒有在 --instances 裡逐機指定時的預設 profile(vtol / copter / plane)"),
    Knob("source_system", "GZ_VTOL_SYSID", 255, int,
         "本端 sysid。255 是 GCS 慣例;⚠️ 不要撞到機體的 sysid"),
    Knob("connect_timeout", "GZ_VTOL_CONNECT_TIMEOUT", 20.0, float,
         "等第一個心跳的秒數。⚠️ SERIAL0 帶 :wait 時 SITL 卡在 accept(),"
         "我們連上才會放行,所以第一次會偏慢"),
    Knob("stream_hz", "GZ_VTOL_STREAM_HZ", 5.0, float,
         "位置/姿態/HUD 這類快訊息要求的頻率"),
    Knob("slow_stream_hz", "GZ_VTOL_SLOW_STREAM_HZ", 2.0, float,
         "SYS_STATUS / EXTENDED_SYS_STATE 這類慢訊息要求的頻率"),
    Knob("rate_window", "GZ_VTOL_RATE_WINDOW", 25.0, float,
         "量測實際頻率的視窗秒數。⚠️ 太短會被 lock_step 的 RTF 抖動騙過去"
         "(事實【1】四機時 RTF 只有 0.236)。"
         "⚠️⚠️ 2026-09-10 實測把預設從 5 秒改成 25 秒:視窗要能裝下**最慢那個訊息**"
         "的好幾個週期,而最慢的是 HEARTBEAT(模擬時間 1 Hz)。"
         "期望則數 = 1 Hz × sim_rtf × 視窗;RTF 0.16 配 5 秒窗只有 **0.8 則**,"
         "於是量到 0.00 Hz、門檻判紅,而飛機**完全健康**(其餘每一項都是門檻的兩倍)。"
         "25 秒窗在同樣 RTF 下期望 4 則,足以判定。"
         "⇒ 這是**取樣不足**造成的假紅燈,不是放寬門檻能解的;"
         "把 rate_floor_frac 調鬆反而會讓真的沒串流也變綠。"
         "RTF 更低(更多架、開 GUI)時要再拉長,算式如上。"),
    Knob("rate_floor_frac", "GZ_VTOL_RATE_FLOOR_FRAC", 0.5, float,
         "實測頻率 / 要求頻率 的下限。⚠️⚠️ 這個比值**必須先除以量到的 sim_rtf** 再比,"
         "否則它是一個保證誤報的門檻:串流是照**模擬時間**排的,lock_step 下 RTF 0.236"
         "(事實【1】四機實測)⇒ 要求 5 Hz 的訊息在牆鐘上只會有 1.18 Hz,"
         "而 5×0.5=2.5 的門檻會把**完全健康**的機隊判紅。"
         "本檔比的是 `牆鐘頻率 / sim_rtf`(見 step_streams),raw 值照樣印出來"),
    Knob("hb_floor_hz", "GZ_VTOL_HB_FLOOR_HZ", 0.2, float,
         "HEARTBEAT 的頻率下限(同樣先除以 sim_rtf 正規化後再比)。"
         "⚠️ 原版把它寫成模組常數 0.5 且**不可覆寫** —— 而 ArduPilot 的心跳是 1 Hz "
         "**模擬時間**,RTF 0.236 時牆鐘只有 0.236 Hz ⇒ 0.5 的門檻在健康機隊上必紅。"
         "門檻壓低是刻意的:這一欄抓的是『心跳停了』,不是抓『慢』"),
    Knob("rtf_stale_frac", "GZ_VTOL_RTF_STALE_FRAC", 0.4, float,
         "⚠️⚠️ time_boot_ms 允許多久沒更新(× rate_window)。"
         "**沒有這一條的話 sim_rtf 是一個可以被單一取樣騙過去的門檻**:"
         "視窗開頭收到一則 ATTITUDE、之後整條鏈路凍住,原版用"
         "`(boot_ms−b0)/(最後一次更新的牆鐘−起點)` 當分母 ⇒ 算出來是那 10 ms 的瞬時速率,"
         "可以是 20.0 而完全看不出後面 5 秒是死的。本檔改成除以**整個視窗**,"
         "並額外要求最後一次更新不能比 rate_window×這個比例還舊"),
    Knob("allow_foreign_sysid", "GZ_VTOL_ALLOW_FOREIGN_SYSID", False, _b,
         "⚠️⚠️ 同一條鏈路上出現第二個 sysid ⇒ 那個埠不是某一架 SITL 的 SERIAL,"
         "而是 MAVProxy / fanout 之類的共用埠 ⇒ **我們不知道自己在對誰下令**。"
         "預設判**紅**。原版只在細節字串裡加一行 ⚠️ 而那一列照樣是 ✔ —— "
         "警告埋在綠燈裡等於沒有警告(事實【2】:埠被偷走在 ss 那一層是隱形的,"
         "MAVLink 層的這個影子是我們唯一看得見的證據)。"
         "真的要接 fanout 埠才開它"),
    Knob("min_sim_rtf", "GZ_VTOL_MIN_SIM_RTF", 0.01, float,
         "⚠️⚠️ 逐機模擬時鐘前進速率(time_boot_ms 增量 / 牆鐘增量)下限。"
         "**這是本檔最重要的一個門檻**:它是唯一能分辨『那一架的 lock_step 卡死』"
         "與『整個 world 只是慢』的量。卡死的那架這一欄會是 0.000,而 /stats 的 "
         "sim_time 照樣前進(事實【3】)。門檻壓很低是刻意的 —— 我們要抓的是**停**"),
    Knob("eof_limit", "GZ_VTOL_EOF_LIMIT", 3, int,
         "mavtcp handle_eof 觸發幾次就判定鏈路死亡。"
         "⚠️ mavutil.py:1281-1282 EAGAIN 走 `return \"\"` 不會碰到 handle_eof "
         "⇒ 一次其實就已經是結論;3 只是留餘裕。重點是**停下來**,"
         "不是忍受它把 151 MB 的 `EOF on TCP socket` 噴進 log"),
    Knob("cmd_timeout", "GZ_VTOL_CMD_TIMEOUT", 3.0, float,
         "單一指令/參數等回應的總預算(秒)。⚠️ 這是**全隊共用**的預算,"
         "不是逐機各付一次 —— 見 §2.3"),
    Knob("cmd_retries", "GZ_VTOL_CMD_RETRIES", 2, int, "指令/參數重送次數"),
    Knob("param_tol", "GZ_VTOL_PARAM_TOL", 1e-4, float,
         "PARAM_SET 讀回比對的相對容差(比照 mavlink_link.py:1632 的 tol 語意)"),
    Knob("takeoff_alt", "GZ_VTOL_TAKEOFF_ALT", 30.0, float,
         "VTOL 起飛目標高度(公尺,相對 HOME)"),
    Knob("takeoff_frac", "GZ_VTOL_TAKEOFF_FRAC", 0.90, float,
         "到達判準:relative_alt ≥ 目標 × 這個比例"),
    Knob("takeoff_timeout", "GZ_VTOL_TAKEOFF_TIMEOUT", 120.0, float,
         "爬到目標高度的逾時(牆鐘秒)。⚠️ lock_step + RTF 0.236 ⇒ "
         "牆鐘要留模擬時間的 4 倍以上"),
    Knob("land_timeout", "GZ_VTOL_LAND_TIMEOUT", 180.0, float, "等待落地上鎖的逾時(牆鐘秒)"),
    Knob("origin_lat", "GZ_VTOL_ORIGIN_LAT", 25.0777, float,
         "world 的 spherical_coordinates 原點緯度(taoyuan_runway.sdf:51)"),
    Knob("origin_lon", "GZ_VTOL_ORIGIN_LON", 121.2325, float,
         "world 原點經度(taoyuan_runway.sdf:52)"),
    Knob("origin_alt", "GZ_VTOL_ORIGIN_ALT", 32.0, float,
         "world 原點 elevation(taoyuan_runway.sdf:53)。"
         "⚠️ 未解問題:地面在世界 z=0 而這裡寫 32 ⇒ Gazebo 的『海平面』其實是 32 m AMSL"),
    Knob("origin_tol_m", "GZ_VTOL_ORIGIN_TOL_M", 300.0, float,
         "HOME 與 world 原點的水平容差(公尺)。"
         "⚠️ 這一項抓的是『--home 打錯 / 忘了給』這種**量級**錯誤,不是精度"),
    Knob("origin_alt_tol_m", "GZ_VTOL_ORIGIN_ALT_TOL_M", 60.0, float,
         "HOME 與 world 原點的高度容差(公尺)。放寬到 60 是因為上面那個 32 m 的問題"),
    Knob("watch_seconds", "GZ_VTOL_WATCH_SECONDS", 30.0, float, "`watch` 子指令要看多久"),
    Knob("watch_interval", "GZ_VTOL_WATCH_INTERVAL", 1.0, float, "`watch` 每幾秒印一列"),
    Knob("strict", "GZ_VTOL_STRICT", True, _b,
         "任何一項失敗就以非 0 結束。⚠️ 降級成 0 會被**大聲印出來** —— "
         "靜默降級是這一整類 bug 的成因(比照 contract_verify.py 的 STRICT)"),
    Knob("param_check", "GZ_VTOL_PARAM_CHECK", True, _b,
         "是否跑逐機參數契約。關掉會大聲印出來"),
)


@dataclass
class Config:
    values: dict[str, Any] = field(default_factory=dict)
    sources: dict[str, str] = field(default_factory=dict)

    def __getattr__(self, name: str) -> Any:      # cfg.stream_hz
        try:
            return self.values[name]
        except KeyError:
            raise AttributeError(name) from None

    def lines(self) -> list[str]:
        out = []
        for k in KNOBS:
            v = self.values.get(k.cli)
            out.append(f"  {k.cli:<18} = {v!r:<24} [{self.sources.get(k.cli, '?')}]  ({k.env})")
        return out

    def to_dict(self) -> dict[str, Any]:
        return {k.cli: {"value": self.values.get(k.cli), "source": self.sources.get(k.cli),
                        "env": k.env} for k in KNOBS}


def resolve_config(args: argparse.Namespace) -> Config:
    """優先序 cli > env > default,並**記住是誰給的**。

    ⚠️ 「記住來源」不是裝飾。本專案栽在同一個坑很多次:開關看起來開著,
       但沒有任何一行證明它生效(memory: sortie-evidence-is-volatile)。
    """
    cfg = Config()
    for k in KNOBS:
        cli_val = getattr(args, k.cli, None)
        if cli_val is not None:
            cfg.values[k.cli] = k.cast(cli_val) if isinstance(cli_val, str) else cli_val
            cfg.sources[k.cli] = "cli"
            continue
        env_val = os.environ.get(k.env)
        if env_val is not None and env_val != "":
            try:
                cfg.values[k.cli] = k.cast(env_val)
                cfg.sources[k.cli] = "env"
                continue
            except (TypeError, ValueError) as e:
                raise SystemExit(f"✗ 環境變數 {k.env}={env_val!r} 轉型失敗:{e}")
        cfg.values[k.cli] = k.default
        cfg.sources[k.cli] = "default"
    return cfg


# ══════════════════════════════════════════════════════════════════════
# 二、逐機參數契約表(形狀比照 scripts/wsl/sar_swarm_patches/contract_table.py)
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class ParamContract:
    """一個要從**飛控讀回**比對的參數。

    lo/hi 為 None = 該側不設限(只記錄)。
    `fallback` 是「餵值路徑斷掉時會靜靜退回去的那個值」——
    命中它一律視為「沒有人設過它」,**與區間檢查獨立判定**。
    contract_table.py 把這一欄叫「泡棉禁區」,同一個概念。
    """
    name: str
    lo: float | None
    hi: float | None
    fallback: float | None
    why: str
    evidence: str


#: SRV_Channel 功能碼(~/ardupilot/libraries/SRV_Channel/SRV_Channel.h,2026-09-06 逐行確認)
K_AILERON, K_ELEVATOR, K_RUDDER, K_THROTTLE = 4, 19, 21, 70          # :53 :68 :70 :114
K_MOTOR1, K_MOTOR2, K_MOTOR3, K_MOTOR4 = 33, 34, 35, 36              # :82-85

_SDF_PWM_EVID = (
    "iris 的四個 <control> 都是 servo_min 1100 / servo_max 1900,而 quadplane.parm:69-70 是 "
    "Q_M_PWM_MIN/MAX 1000/2000。照抄 iris ⇒ ArduPilotPlugin.cc:1673-1675 的 "
    "raw_cmd=(pwm-1100)/800 會把底部 12.5% 全夾成 0、頂部 12.5% 全夾成 1,四顆一起損失 25% 行程 —— "
    "而飛機只是『推力怪怪的』,不會報任何錯。"
    "⚠️ Q_M_PWM_TYPE 預設 NORMAL ⇒ SERVOn_MIN/MAX 對馬達輸出**不生效**,生效的是 Q_M_PWM_MIN/MAX"
)

_SDF_CHAN_EVID = (
    "端到端鏈是 SERVO(N+1) → servos[N] → pkt.pwm[N] → SDF <control channel=\"N\">。"
    "ArduPlane 預設 SERVO1-4 = 副翼/升降舵/前推油門/方向舵 ⇒ SDF channel 0/1/2/3;"
    "QuadPlane 升力馬達由 quadplane.cpp:667-671 setup_default_channels() 把 motor1..4 "
    "指到 CH_5..CH_8 ⇒ SDF channel 4/5/6/7。"
    "⚠️ --model JSON 時 SIM_QuadPlane 不會被實例化,真正生效的是 SERVOn_FUNCTION —— "
    "所以這一欄**必須讀回**,不能相信『我有寫進 parm 檔』"
)

PROFILES: dict[str, tuple[ParamContract, ...]] = {
    # ── VTOL(QuadPlane on ArduPlane)─────────────────────────────────
    "vtol": (
        ParamContract(
            "Q_ENABLE", 1, 1, 0,
            "⚠️⚠️ **這是判斷『這台到底是不是 VTOL』的唯一硬證據**。"
            "HEARTBEAT.type 沒有用:quadplane.cpp:4042-4048 在 Q_MAV_TYPE=0(預設)時"
            "一律回 MAV_TYPE_FIXED_WING ⇒ QuadPlane 在 MAVLink 上和純固定翼長得一模一樣。"
            "Q_ENABLE=0 ⇒ 整台退回純固定翼:起飛指令會被 "
            "GCS_MAVLink_Plane.cpp:899-901 `!quadplane.available()` 擋掉回 MAV_RESULT_FAILED",
            "quadplane.parm:48 Q_ENABLE 1;quadplane.cpp:4042-4048;GCS_MAVLink_Plane.cpp:899-901",
        ),
        # ⚠️ 期望值同樣**不是常數**:gz_vtol_world.py 的 LIFT_PWM_MIN_DEFAULT/MAX_DEFAULT
        #    雖然預設 1000/2000,但可被 GZ_VTOL_LIFT_PWM_MIN/MAX 覆寫(gz_vtol_world.py:
        #    1594-1595 / 1654-1657)。把 1000 抄進這裡 ⇒ 那邊改了這邊靜靜不動,
        #    而症狀是「參數契約說沒問題,飛起來推力怪怪的」。
        #    ⇒ 由 --ports-json 逐機讀回真正的 servo_min/servo_max;沒有就判未驗證。
        ParamContract("Q_M_PWM_MIN", None, None, None,
                      "⚠️⚠️ 必須與**那一架**SDF 升力馬達 <control> 的 servo_min 對齊;"
                      "期望值由 --ports-json 提供,不寫死", _SDF_PWM_EVID),
        ParamContract("Q_M_PWM_MAX", None, None, None,
                      "⚠️⚠️ 必須與**那一架**SDF 升力馬達 <control> 的 servo_max 對齊;"
                      "期望值由 --ports-json 提供,不寫死", _SDF_PWM_EVID),
        ParamContract("Q_M_PWM_TYPE", 0, 0, None,
                      "NORMAL(0)。⚠️ 一旦改成別的,上面兩條與 SDF 的對齊推論就不成立了",
                      _SDF_PWM_EVID),
        ParamContract("SERVO5_FUNCTION", K_MOTOR1, K_MOTOR1, 0,
                      "升力馬達 1 ⇒ SDF channel 4", _SDF_CHAN_EVID),
        ParamContract("SERVO6_FUNCTION", K_MOTOR2, K_MOTOR2, 0,
                      "升力馬達 2 ⇒ SDF channel 5", _SDF_CHAN_EVID),
        ParamContract("SERVO7_FUNCTION", K_MOTOR3, K_MOTOR3, 0,
                      "升力馬達 3 ⇒ SDF channel 6", _SDF_CHAN_EVID),
        ParamContract("SERVO8_FUNCTION", K_MOTOR4, K_MOTOR4, 0,
                      "升力馬達 4 ⇒ SDF channel 7", _SDF_CHAN_EVID),
        ParamContract("SERVO1_FUNCTION", K_AILERON, K_AILERON, 0,
                      "副翼 ⇒ SDF channel 0", _SDF_CHAN_EVID),
        ParamContract("SERVO2_FUNCTION", K_ELEVATOR, K_ELEVATOR, 0,
                      "升降舵 ⇒ SDF channel 1", _SDF_CHAN_EVID),
        ParamContract("SERVO3_FUNCTION", K_THROTTLE, K_THROTTLE, 0,
                      "**前推馬達** ⇒ SDF channel 2(不是升力馬達,最常搞錯的一格)",
                      _SDF_CHAN_EVID),
        ParamContract("SERVO4_FUNCTION", K_RUDDER, K_RUDDER, 0,
                      "方向舵 ⇒ SDF channel 3", _SDF_CHAN_EVID),
        ParamContract("ARSPD_USE", 1, 1, 0,
                      "空速計入迴路。⚠️ 關掉的話 Q_ASSIST_SPEED 這道救援閘門也跟著失去輸入",
                      "quadplane.parm:4 ARSPD_USE 1"),
        ParamContract("Q_ASSIST_SPEED", 1, 40, 0,
                      "固定翼段低於此空速就叫升力馬達幫忙。0 = 關閉 ⇒ 失速無救",
                      "quadplane.parm:47 Q_ASSIST_SPEED 18"),
        ParamContract("SCHED_LOOP_RATE", 200, 400, None,
                      "⚠️ 這個參數**與機型相關**,而且 ArduPilot 4.7/4.8 對 --defaults 裡"
                      "未知的參數名是**靜默略過**的 ⇒ 只有讀回才知道到底吃到什麼",
                      "quadplane.parm:64 SCHED_LOOP_RATE 300;memory: sitl-param-renames-silent"),
    ),
    # ── 純多旋翼(ArduCopter;同一個 world 混機種時會出現)───────────
    "copter": (
        ParamContract("SERVO1_FUNCTION", K_MOTOR1, K_MOTOR1, 0,
                      "⚠️ copter 的馬達從 SERVO1 起(plane 是從 SERVO5 起)"
                      "⇒ **同一份 SDF channel 對照表不能兩種機體共用**",
                      "SRV_Channel.h:82-85 k_motor1..k_motor4 = 33..36"),
        ParamContract("SERVO2_FUNCTION", K_MOTOR2, K_MOTOR2, 0, "馬達 2", "同上"),
        ParamContract("SERVO3_FUNCTION", K_MOTOR3, K_MOTOR3, 0, "馬達 3", "同上"),
        ParamContract("SERVO4_FUNCTION", K_MOTOR4, K_MOTOR4, 0, "馬達 4", "同上"),
        # ⚠️⚠️⚠️ 這兩條原本寫成 `lo=hi=1000/2000、fallback=1100/1900`,而那是**反的**,
        #   會在唯一真的壞掉的情況下亮綠燈:
        #     上游 iris 的 SDF <control> 是 servo_min 1100 / servo_max 1900
        #       (gz_vtol_world.py:255-265 與 :318-325,而且該檔明文寫「本檔不擅自改
        #        別人的模型,只在 ports.json 把實際刻度讀回來報告」),
        #     ArduCopter 的 MOT_PWM_MIN/MAX **預設就是 1000/2000**。
        #   ⇒ 讀回 1000/2000 = 兩側刻度**對不上** = 事實【7】那個 25% 行程損失
        #     ——原版判它「通過」;而讀回 1100(= 真正對得上 SDF 的值)原版判它
        #     「命中退回值 ⇒ 違規」。綠燈與紅燈整組接反。
        #   ⇒ 期望值不可寫死:改由 --ports-json 逐機讀那一架自己的 servo_min/servo_max
        #     (見 apply_ports_json)。沒有 ports.json 時本檔**不猜**,判「未驗證 ⇒ 違規」,
        #     理由與「讀不到 = 違規,不是跳過」同一條。
        ParamContract("MOT_PWM_MIN", None, None, None,
                      "⚠️⚠️ 期望值 = 那一架 SDF 升力通道的 servo_min,由 --ports-json 提供;"
                      "**不是**常數。ArduCopter 預設 1000 與上游 iris SDF 的 1100 對不上",
                      _SDF_PWM_EVID),
        ParamContract("MOT_PWM_MAX", None, None, None,
                      "⚠️⚠️ 期望值 = 那一架 SDF 升力通道的 servo_max,由 --ports-json 提供",
                      _SDF_PWM_EVID),
        ParamContract("SCHED_LOOP_RATE", 200, 500, None,
                      "copter 預設 400", "memory: sitl-param-renames-silent"),
    ),
    # ── 純固定翼(沒有 quadplane)────────────────────────────────────
    "plane": (
        ParamContract("Q_ENABLE", 0, 0, None,
                      "宣告 plane 就**不該**有 quadplane。讀回是 1 表示 profile 宣告錯了",
                      "quadplane.cpp:4042-4048"),
        ParamContract("SERVO1_FUNCTION", K_AILERON, K_AILERON, 0, "副翼 ⇒ SDF channel 0",
                      _SDF_CHAN_EVID),
        ParamContract("SERVO2_FUNCTION", K_ELEVATOR, K_ELEVATOR, 0, "升降舵 ⇒ SDF channel 1",
                      _SDF_CHAN_EVID),
        ParamContract("SERVO3_FUNCTION", K_THROTTLE, K_THROTTLE, 0, "推力 ⇒ SDF channel 2",
                      _SDF_CHAN_EVID),
        ParamContract("SERVO4_FUNCTION", K_RUDDER, K_RUDDER, 0, "方向舵 ⇒ SDF channel 3",
                      _SDF_CHAN_EVID),
        ParamContract("SCHED_LOOP_RATE", 200, 400, None, "plane 預設 300",
                      "memory: sitl-param-renames-silent"),
    ),
}

#: profile → (起飛模式, 降落模式)。⚠️ 見 §4.3:降落只能走切模式。
PROFILE_MODES: dict[str, tuple[str, str]] = {
    "vtol":   ("GUIDED", "QLAND"),
    "plane":  ("GUIDED", "RTL"),      # 純固定翼沒有 QLAND;RTL 是最接近「收尾」的可讀回模式
    "copter": ("GUIDED", "LAND"),
}

#: profile → 顯式 COMMAND_INT 起飛時要用的 (frame, z 的正負號)。見 §4.2。
PROFILE_TAKEOFF_INT: dict[str, tuple[int, float]] = {
    # GCS_MAVLink_Plane.cpp:892-897 只收 LOCAL_OFFSET_NED,:894 takeoff_alt = -z
    "vtol":   (_mav.MAV_FRAME_LOCAL_OFFSET_NED, -1.0),
    "plane":  (_mav.MAV_FRAME_LOCAL_OFFSET_NED, -1.0),
    # GCS_MAVLink_Copter.cpp:580-582 只收 GLOBAL_RELATIVE_ALT(=3,**不是** _INT=6),:590 用 +z
    "copter": (_mav.MAV_FRAME_GLOBAL_RELATIVE_ALT, +1.0),
}

#: 要串流的訊息:(訊息 id, 名稱, 是否算「快」訊息)
STREAM_MSGS: tuple[tuple[int, str, bool], ...] = (
    (_mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT", True),
    (_mav.MAVLINK_MSG_ID_ATTITUDE, "ATTITUDE", True),
    (_mav.MAVLINK_MSG_ID_VFR_HUD, "VFR_HUD", True),
    (_mav.MAVLINK_MSG_ID_SYS_STATUS, "SYS_STATUS", False),
    (_mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, "EXTENDED_SYS_STATE", False),
)

#: ⚠️ HEARTBEAT 故意**不**列進 STREAM_MSGS:我們只量它、絕不去設它的間隔。
#:    把 HEARTBEAT 的 interval 設成 -1 會把整條鏈路變成瞎子,而且看起來像「飛控掛了」。
#: ⚠️ 心跳的頻率門檻已經搬進 KNOBS 的 `hb_floor_hz`(可覆寫、會印出生效值與來源)。
#:    原本寫死在這裡的 0.5 有兩個問題:(1) 任務書要求「每個可調常數都要能覆寫」,
#:    (2) 0.5 在 lock_step RTF 0.236 下對**健康**機隊必紅。

#: 這兩條參數契約的期望值不是常數,而是**那一架自己 SDF 裡的 servo_min/servo_max**
#: (gz_vtol_world.py 會把逐通道刻度寫進 `<world>.ports.json` 的 vehicles[].channels)。
#: 寫死在這裡就是「同一個常數活在兩個地方」——而且 gz_vtol_world.py 的
#: LIFT_PWM_MIN_DEFAULT/MAX_DEFAULT 還可以被 GZ_VTOL_LIFT_PWM_MIN/MAX 覆寫,
#: 改了那邊這邊不會有任何反應。⇒ 見 apply_ports_json() 與 step_params 的 per-link 覆寫。
PWM_CONTRACT_NAMES: dict[str, tuple[str, str]] = {
    "vtol": ("Q_M_PWM_MIN", "Q_M_PWM_MAX"),
    "copter": ("MOT_PWM_MIN", "MOT_PWM_MAX"),
}


def _result_name(res: Any) -> str:
    """MAV_RESULT → 名字。⚠️ 只印數字的話 3(UNSUPPORTED)與 4(FAILED)看起來一樣無害。

    ⚠️ 2026-09-10 實測踩到:呼叫端有兩種,一種傳 int,一種直接傳整個
    COMMAND_ACK 訊息(set_mode 的 acks 字典裝的是訊息物件)。原版只吃 int,
    於是 set_mode 走到這裡就 TypeError,整支任務在解鎖那一步炸掉。
    這裡兩種都收 —— 型別分歧修在**這一端**比較安全:漏改一個呼叫端的代價是
    整趟航次中斷,而中斷點離真正的原因很遠。
    """
    if res is None:
        return "無 ACK"
    if hasattr(res, "result"):          # COMMAND_ACK 訊息物件
        res = res.result
    try:
        num = int(res)
    except (TypeError, ValueError):
        return f"無法解讀的 ACK({res!r})"
    e = _mav.enums.get("MAV_RESULT", {})
    v = e.get(num)
    return f"{v.name}({num})" if v is not None else f"MAV_RESULT_{num}"


# ══════════════════════════════════════════════════════════════════════
# 三、EOF 守衛
# ══════════════════════════════════════════════════════════════════════
@dataclass
class EofGuard:
    """把 pymavlink `mavtcp.handle_eof` 換掉的計數器。

    ⚠️ 原版 mavutil.py:1268-1271 是 `print("EOF on TCP socket"); self.reconnect()`,
       而 mavtcp.recv():1286-1287 在對端關閉後**每次呼叫都會走到那裡**。
       配上 recv_match(blocking=True) 的 `self.select(timeout/2)`(:526)——
       已關閉的 socket select 立刻返回可讀 ⇒ 空轉狂噴。本專案實測噴過 151 MB。

    ⚠️ 同時我們也**放棄** autoreconnect。理由:悄悄重連會把「SITL 死了」偽裝成
       「有點卡」,而這條線上最貴的錯誤就是把死掉讀成活著。
    """
    tag: str
    limit: int
    count: int = 0
    first_epoch: float | None = None

    def install(self, conn: Any) -> None:
        if not hasattr(conn, "handle_eof"):
            return                       # udp 連線沒有這個方法,不是錯
        def _handle_eof() -> None:       # noqa: ANN202  綁在 instance 上,蓋過 class method
            self.count += 1
            if self.first_epoch is None:
                self.first_epoch = time.time()
                sys.stderr.write(
                    f"[eof] {self.tag}: TCP 對端已關閉(pymavlink mavutil.py:1286-1287)。"
                    f"再 {self.limit - 1} 次就判定鏈路死亡並停止讀取。\n")
            elif self.count == self.limit:
                sys.stderr.write(f"[eof] {self.tag}: 達到上限 {self.limit},停止讀取。\n")
        conn.handle_eof = _handle_eof

        # ⚠️ 原版只蓋掉 handle_eof,漏了同一個類別上的 `handle_disconnect`
        #    (mavutil.py:1263-1265,ECONNRESET / EPIPE 走這裡):它也是
        #    `print(...); self.reconnect()`,同樣是無條件 print 到 stdout ——
        #    而 stdout 正是人看報表的地方,一行雜訊插在逐機表格中間比在 stderr 更礙事。
        #    (reconnect() 本身因為 autoreconnect=False 是 no-op,mavutil.py:1306-1312,
        #     所以「悄悄重連」那一半確實已經關掉了;這裡補的是輸出那一半。)
        if hasattr(conn, "handle_disconnect"):
            def _handle_disconnect() -> None:   # noqa: ANN202
                self.count += 1
                if self.first_epoch is None:
                    self.first_epoch = time.time()
                    sys.stderr.write(
                        f"[eof] {self.tag}: TCP 對端 reset/EPIPE"
                        f"(pymavlink mavutil.py:1263-1265)。\n")
            conn.handle_disconnect = _handle_disconnect

    @property
    def tripped(self) -> bool:
        return self.count >= self.limit


# ══════════════════════════════════════════════════════════════════════
# 四、單機鏈路
# ══════════════════════════════════════════════════════════════════════
class Link:
    """一架 SITL 的 TCP 鏈路 + 訊息快取 + 頻率統計。

    ⚠️ 本類別**從不呼叫** `recv_match(blocking=True)` —— 見 EofGuard 的說明。
       所有等待都由 `Fleet.pump()` 用 selectors 統一驅動。
    """

    def __init__(self, instance: int, profile: str, cfg: Config,
                 port: int | None = None) -> None:
        self.instance = int(instance)
        self.profile = profile
        self.cfg = cfg
        self.key = f"I{self.instance}"
        # SERIAL0→base+0、SERIAL1→base+2、SERIAL2→base+3(SITL_State.h:29-38)
        serial_off = {0: 0, 1: 2, 2: 3, 5: 5, 6: 6, 7: 7, 8: 8}.get(int(cfg.serial))
        if serial_off is None:
            raise SystemExit(f"✗ --serial {cfg.serial} 不是 TCP 序列埠;可用 0/1/2/5/6/7/8"
                             f"(SITL_State.h:29-38,索引 3/4 不是 tcp)")
        if port is not None:
            # 顯式埠(`@port` 或 --from-state):**完全繞過算術**。
            # ⚠️ 啟動器只要用過 `--base-port`,任何算術都是猜的(檔頭 §2.5)。
            self.port = int(port) + serial_off
            self.port_src = "顯式"
        else:
            self.port = (int(cfg.tcp_base)
                         + int(cfg.tcp_stride) * (self.instance - int(cfg.instance_base))
                         + serial_off)
            self.port_src = (f"算術 {cfg.tcp_base}+{cfg.tcp_stride}×"
                             f"({self.instance}−{cfg.instance_base})+{serial_off}")
        if self.port <= 0 or self.port > 65535:
            raise SystemExit(
                f"✗ {self.key} 算出來的埠是 {self.port},不是合法 TCP 埠。"
                f"({self.port_src})⇒ 多半是 --instance-base / --tcp-base 沒對上")
        self.conn_str = f"tcp:{cfg.host}:{self.port}"

        self.conn: Any = None
        self.eof = EofGuard(self.key, int(cfg.eof_limit))
        self.dead = False
        self.dead_reason = ""

        #: ⚠️ sysid **不可以假設等於索引**。這裡一律從 HEARTBEAT.srcSystem 讀回。
        self.sysid: int | None = None
        self.compid: int | None = None
        self.hb_type: int | None = None
        self.hb_autopilot: int | None = None
        self.hb_msg: Any = None
        self.hb_epoch: float | None = None
        #: ⚠️ 同一條鏈路上出現第二個 sysid = 有東西在做 fanout / 埠共用。
        #:    事實【2】說埠衝突在 `ss` 這一層是隱形的,這一欄是它在 MAVLink 層的影子。
        self.foreign_sysids: dict[int, int] = {}
        #: 本機 pymavlink 解不出來的訊息型別計數。留著是為了不讓它們
        #: 「靜靜消失」—— 數量異常大代表韌體與 pymavlink 版本差太多。
        self.undecodable: dict[str, int] = {}

        self.latest: dict[str, Any] = {}
        self.counts: dict[str, int] = {}
        self.statustexts: list[tuple[float, int, str]] = []
        self.bad_data = 0

        #: 逐機模擬時鐘(見 §一 (c))
        self.boot_ms: int | None = None
        self.boot_ms_epoch: float | None = None

        self._want: tuple[set[str], Callable[[Any], bool] | None] | None = None
        self._got: Any = None

        #: 這一架 SDF 升力通道的 (servo_min, servo_max)。由 --ports-json 填。
        #: ⚠️ None = **沒有權威來源**。這種情況下 PWM 那兩條契約判「未驗證 ⇒ 違規」,
        #:    不是判過 —— 猜一個常數正是它原本壞掉的方式(見 PWM_CONTRACT_NAMES)。
        self.pwm_expect: tuple[float, float] | None = None
        self.pwm_src: str = "(無 --ports-json ⇒ 未驗證)"

    # -- 連線 --------------------------------------------------------
    def connect(self) -> str | None:
        """回 None = 成功,否則回錯誤字串。"""
        try:
            self.conn = _mavutil.mavlink_connection(
                self.conn_str,
                source_system=int(self.cfg.source_system),
                source_component=_mav.MAV_COMP_ID_MISSIONPLANNER,
                # ⚠️ 明確關掉:悄悄重連會把「SITL 死了」偽裝成「有點卡」
                autoreconnect=False,
                retries=0,
            )
        except Exception as e:                                   # noqa: BLE001
            self.dead, self.dead_reason = True, f"連不上 {self.conn_str}: {e!r}"
            return self.dead_reason
        self.eof.install(self.conn)
        return None

    @property
    def fd(self) -> int | None:
        c = self.conn
        if c is None:
            return None
        f = getattr(c, "fd", None)
        if f is not None:
            return int(f)
        p = getattr(c, "port", None)
        return int(p.fileno()) if p is not None else None

    def close(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:                                    # noqa: BLE001
                pass
            self.conn = None

    # -- 收訊 --------------------------------------------------------
    def expect(self, types: Sequence[str], pred: Callable[[Any], bool] | None = None) -> None:
        self._want = (set(types), pred)
        self._got = None

    @property
    def got(self) -> Any:
        return self._got

    def drain(self, max_msgs: int = 400) -> int:
        """非阻塞讀到沒有為止。回讀進來的訊息數。"""
        n = 0
        while n < max_msgs:
            if self.eof.tripped:
                self._die(f"TCP EOF × {self.eof.count}(對端關閉;SITL 可能已退出)")
                return n
            try:
                msg = self.conn.recv_match(blocking=False)
            except (ConnectionResetError, OSError) as e:
                self._die(f"socket 錯誤 {e!r}")
                return n
            if msg is None:
                if self.eof.tripped:
                    self._die(f"TCP EOF × {self.eof.count}(對端關閉;SITL 可能已退出)")
                return n
            self._cache(msg)
            n += 1
        return n

    def _die(self, reason: str) -> None:
        if not self.dead:
            self.dead, self.dead_reason = True, reason

    def _cache(self, msg: Any) -> None:
        t = msg.get_type()
        if t == "BAD_DATA":
            self.bad_data += 1
            return
        # ⚠️⚠️ 本機 pymavlink 解不出來的訊息一律**不列入 sysid 普查**。
        #   2026-09-10 實測:ArduPlane 會送 msgid 295(AVAILABLE_MODES,4.6+ 才有),
        #   而本機 pymavlink 2.4.49 沒有它的定義 ⇒ 解成 `UNKNOWN_295`、
        #   `get_srcSystem()` 回 **0** ⇒ 外來 sysid 守衛報 [0],把一條乾淨的
        #   直連埠誤判成「MAVProxy 共用埠」。那是**假陽性**,而且它會讓
        #   step_streams 整條紅掉,把人導去查埠而不是查飛機。
        #   ⚠️ 這不會削弱守衛:真的有第二台載具時,它送的是解得出來的訊息。
        if t.startswith("UNKNOWN_"):
            self.undecodable[t] = self.undecodable.get(t, 0) + 1
            return
        src, comp = msg.get_srcSystem(), msg.get_srcComponent()

        if t == "HEARTBEAT" and msg.type != _mav.MAV_TYPE_GCS and src != int(self.cfg.source_system):
            # ⚠️ 只認「非 GCS 且不是自己」的心跳。做法抄自 mavlink_link.py:1474-1481。
            if self.sysid is None:
                self.sysid, self.compid = src, comp
                self.hb_type, self.hb_autopilot = int(msg.type), int(msg.autopilot)
                self.conn.target_system, self.conn.target_component = src, comp
            if src == self.sysid:
                self.hb_msg, self.hb_epoch = msg, time.time()

        if self.sysid is not None and src != self.sysid:
            self.foreign_sysids[src] = self.foreign_sysids.get(src, 0) + 1
            return

        self.counts[t] = self.counts.get(t, 0) + 1
        self.latest[t] = msg

        # 逐機模擬時鐘:優先 ATTITUDE(沒有 GPS 也照送),退而求其次 GLOBAL_POSITION_INT
        if t in ("ATTITUDE", "GLOBAL_POSITION_INT") and hasattr(msg, "time_boot_ms"):
            if t == "ATTITUDE" or "ATTITUDE" not in self.counts:
                self.boot_ms = int(msg.time_boot_ms)
                # ⚠️⚠️⚠️ 一定要 monotonic：這個值只拿來和 step_streams 的
                #   `t_end = time.monotonic()` 相減求「多久沒更新」。
                #   2026-09-10 實測：原本寫 time.time()（epoch）⇒ 相減得到
                #   **-1788843394.2 秒**，於是 `stale <= stale_budget` 恆為真，
                #   那道「模擬時鐘中途停了」的閘門**結構上不可能響**。
                #   而它印出來的那一欄看起來只是個怪數字，不會有人當成紅燈。
                #   本專案記過同一類：instrument-that-cannot-fire。
                self.boot_ms_epoch = time.monotonic()

        if t == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else bytes(msg.text).decode(errors="replace")
            self.statustexts.append((time.time(), int(msg.severity), txt.rstrip("\x00")))
            if len(self.statustexts) > 400:
                del self.statustexts[:200]

        if self._want is not None:
            types, pred = self._want
            if t in types and (pred is None or pred(msg)):
                self._got, self._want = msg, None

    # -- 便利存取 ----------------------------------------------------
    @property
    def target(self) -> tuple[int, int]:
        """(target_system, target_component)。

        ⚠️⚠️ sysid 還沒讀到時**絕不可以退回 0**:MAVLink 的 target_system=0 是**廣播**。
           「拿不到位址就退回 0」這種寫法會讓一架定不到址的機體把指令打到同鏈路上的
           所有機體 —— 而且完全不報錯。這裡寧可大聲炸掉,由 Fleet.gather 接住並
           **指名是哪一架**。這是本專案「靜默退回預設值」那一整類 bug 的同一個形狀。
        """
        if self.sysid is None:
            raise RuntimeError(
                f"{self.key}: 還沒讀到 sysid,不可定址(target_system=0 是廣播,不是『預設』)")
        return int(self.sysid), int(self.compid or 1)

    @property
    def armed(self) -> bool:
        hb = self.hb_msg
        return bool(hb is not None and (hb.base_mode & _mav.MAV_MODE_FLAG_SAFETY_ARMED))

    @property
    def mode_name(self) -> str:
        hb = self.hb_msg
        if hb is None:
            return "?"
        table = self.mode_by_number()
        return table.get(int(hb.custom_mode), f"MODE{int(hb.custom_mode)}")

    def mode_by_number(self) -> dict[int, str]:
        """⚠️ 模式表**逐機**取:plane 韌體與 copter 韌體的 custom_mode 編號完全不同。

        pymavlink mavutil.py:2567-2569 `mode_mapping_bynumber(mav_type)`;
        :2383-2419 的 AP_MAV_TYPE_MODE_MAP 把 FIXED_WING / VTOL_* 對到 mode_mapping_apm、
        QUADROTOR 等對到 mode_mapping_acm。用寫死的 apm 表去讀 copter 的心跳
        會得到**看起來很合理但完全錯誤**的模式名。
        """
        if self.hb_type is not None:
            m = _mavutil.mode_mapping_bynumber(self.hb_type)
            if m:
                return dict(m)
        return dict(_mavutil.mode_mapping_apm)

    def mode_by_name(self) -> dict[str, int]:
        return {str(v).upper(): int(k) for k, v in self.mode_by_number().items()}

    @property
    def rel_alt_m(self) -> float | None:
        gp = self.latest.get("GLOBAL_POSITION_INT")
        return None if gp is None else gp.relative_alt / 1000.0

    @property
    def latlon(self) -> tuple[float, float] | None:
        gp = self.latest.get("GLOBAL_POSITION_INT")
        return None if gp is None else (gp.lat / 1e7, gp.lon / 1e7)

    def recent_statustext(self, n: int = 5) -> list[str]:
        return [f"[{sev}] {txt}" for _, sev, txt in self.statustexts[-n:]]


# ══════════════════════════════════════════════════════════════════════
# 五、機隊:selectors 驅動,共用總預算
# ══════════════════════════════════════════════════════════════════════
class Fleet:
    """⚠️ 與 `FleetLink.poll()`(mavlink_link.py:1936-1960)的差別是**唯一**的重點:

    那邊是 `for link in links: link.read_telemetry(timeout)` ⇒ 16 架 × 0.5 s = 一輪 8 s。
    這邊是「一次全送出 → 一次 select 等所有 fd → 誰有資料叫醒誰」⇒ 一輪 = 總預算。
    """

    def __init__(self, links: Sequence[Link], cfg: Config) -> None:
        self.links = list(links)
        self.cfg = cfg
        self.sel = selectors.DefaultSelector()
        self._registered: set[int] = set()
        self._last_hb = 0.0

    # -- selector 管理 -----------------------------------------------
    def register(self, link: Link) -> None:
        fd = link.fd
        if fd is None or fd in self._registered:
            return
        self.sel.register(fd, selectors.EVENT_READ, link)
        self._registered.add(fd)

    def retire(self, link: Link) -> None:
        """⚠️⚠️ 死掉的鏈路**一定要**從 selector 上拿掉。

        已關閉的 TCP socket 永遠是 readable ⇒ 留著它 select() 會立刻返回、無限空轉,
        整個 pump 迴圈變成 busy loop。EOF 計數只負責判死,unregister 才是止血。
        """
        fd = link.fd
        if fd is not None and fd in self._registered:
            try:
                self.sel.unregister(fd)
            except (KeyError, ValueError):
                pass
            self._registered.discard(fd)

    @property
    def live(self) -> list[Link]:
        return [l for l in self.links if not l.dead]

    def close(self) -> None:
        for l in self.links:
            l.close()
        try:
            self.sel.close()
        except Exception:                                        # noqa: BLE001
            pass

    # -- 主迴圈 ------------------------------------------------------
    def pump(self, budget_s: float, *, heartbeat: bool = True) -> None:
        """在 `budget_s` 內把所有鏈路上的訊息收乾淨。**一輪的成本是總預算。**"""
        deadline = time.monotonic() + max(0.0, float(budget_s))
        while True:
            if heartbeat:
                self._heartbeat_tick()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            if not self._registered:
                time.sleep(min(remaining, 0.05))
                continue
            for key, _ in self.sel.select(min(remaining, 0.2)):
                link: Link = key.data
                link.drain()
                if link.dead:
                    self.retire(link)

    def _heartbeat_tick(self) -> None:
        """1 Hz 送我們自己的 GCS 心跳(mavlink_link.py:1401-1403 同樣做法)。

        ⚠️ 不送的話飛控可能判 GCS failsafe;而且 MAVProxy 之類的中繼要看到我們才會轉發。
        """
        now = time.monotonic()
        if now - self._last_hb < 1.0:
            return
        self._last_hb = now
        for l in self.live:
            try:
                l.conn.mav.heartbeat_send(_mav.MAV_TYPE_GCS, _mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except (OSError, AttributeError) as e:
                l._die(f"送心跳失敗 {e!r}")
                self.retire(l)

    # -- 平行「送 → 收」 ---------------------------------------------
    def gather(self, links: Sequence[Link], send: Callable[[Link], None],
               types: Sequence[str], pred_of: Callable[[Link], Callable[[Any], bool] | None] | None,
               timeout_s: float, retries: int) -> dict[str, Any]:
        """對多機**同時**送,共用一個總預算收回應。回 `{link.key: msg 或 None}`。

        ⚠️ 這是本檔取代 FleetLink 序列輪詢的核心。重送只重送**還沒回**的那幾架,
           不會因為一架慢就把全隊的預算重跑一遍。
        """
        results: dict[str, Any] = {}
        pending = [l for l in links if not l.dead]
        for _attempt in range(int(retries) + 1):
            if not pending:
                break
            for l in pending:
                l.expect(types, pred_of(l) if pred_of else None)
            for l in pending:
                try:
                    send(l)
                except (OSError, AttributeError, RuntimeError) as e:
                    # RuntimeError 來自 Link.target 的廣播守衛：接住它才能**指名是哪一架**，
                    # 而不是讓整趟跑炸在一個看不出是誰的 traceback 上。
                    l._die(f"送出失敗 {e!r}")
                    self.retire(l)
            deadline = time.monotonic() + float(timeout_s)
            while pending and time.monotonic() < deadline:
                self.pump(min(0.1, max(0.005, deadline - time.monotonic())))
                still: list[Link] = []
                for l in pending:
                    if l.got is not None:
                        results[l.key] = l.got
                    elif l.dead:
                        results[l.key] = None
                    else:
                        still.append(l)
                pending = still
        for l in pending:
            results[l.key] = None
        return results

    def wait_until(self, links: Sequence[Link], ok: Callable[[Link], bool],
                   timeout_s: float, *, tick: Callable[[float], None] | None = None
                   ) -> dict[str, bool]:
        """等到每一架都滿足 `ok`(或逾時)。同樣是**共用**總預算。"""
        deadline = time.monotonic() + float(timeout_s)
        done: dict[str, bool] = {l.key: False for l in links}
        last_tick = 0.0
        while time.monotonic() < deadline:
            self.pump(0.2)
            for l in links:
                if not done[l.key]:
                    if l.dead:
                        done[l.key] = False
                    elif ok(l):
                        done[l.key] = True
            if tick is not None and time.monotonic() - last_tick >= 1.0:
                last_tick = time.monotonic()
                tick(deadline - time.monotonic())
            if all(done[l.key] or l.dead for l in links):
                break
        return done

    # -- 指令 / 參數(全部**讀回**)---------------------------------
    def command_long(self, links: Sequence[Link], command: int, params: Sequence[float],
                     *, timeout_s: float | None = None, retries: int | None = None
                     ) -> dict[str, int | None]:
        """COMMAND_LONG → **讀 COMMAND_ACK**。回 `{key: MAV_RESULT 或 None}`。

        做法抄自 mavlink_link.py:1659-1670,差別是這裡對全隊平行送。
        ⚠️ 「零告警」通常不是飛控沒回,是**我們自己沒讀 ACK**(事實【11】)。
        """
        p = list(params) + [0.0] * (7 - len(params))
        to = float(self.cfg.cmd_timeout if timeout_s is None else timeout_s)
        rt = int(self.cfg.cmd_retries if retries is None else retries)

        def _send(l: Link) -> None:
            l.conn.mav.command_long_send(*l.target, int(command), 0,
                                         *[float(x) for x in p[:7]])

        msgs = self.gather(links, _send, ("COMMAND_ACK",),
                           lambda l: (lambda m: int(m.command) == int(command)), to, rt)
        return {k: (int(v.result) if v is not None else None) for k, v in msgs.items()}

    def command_int(self, links: Sequence[Link], command: int, frame: int,
                    params: Sequence[float], x_deg: float, y_deg: float, z: float,
                    *, timeout_s: float | None = None, retries: int | None = None
                    ) -> dict[str, int | None]:
        """COMMAND_INT → **讀 COMMAND_ACK**。

        ⚠️ `VehicleLink` 沒有這個方法,所以 DO_REPOSITION 在既有資產上**送不出去也檢查不了**
           (事實【11】)。這裡補上。x/y 是 int32 degE7 —— COMMAND_LONG 的 float32 裝不下
           degE7 的 9 位有效數字,精度會掉到公尺級(GCS_Common.cpp:5365-5380 的轉接也救不了,
           因為 wire 上的值本來就已經是 float32 了)。
        """
        p = list(params) + [0.0] * (4 - len(params))
        to = float(self.cfg.cmd_timeout if timeout_s is None else timeout_s)
        rt = int(self.cfg.cmd_retries if retries is None else retries)

        def _send(l: Link) -> None:
            l.conn.mav.command_int_send(
                *l.target, int(frame), int(command), 0, 0,
                float(p[0]), float(p[1]), float(p[2]), float(p[3]),
                int(round(x_deg * 1e7)), int(round(y_deg * 1e7)), float(z))

        msgs = self.gather(links, _send, ("COMMAND_ACK",),
                           lambda l: (lambda m: int(m.command) == int(command)), to, rt)
        return {k: (int(v.result) if v is not None else None) for k, v in msgs.items()}

    def get_params(self, links: Sequence[Link], names: Sequence[str]
                   ) -> dict[str, dict[str, float | None]]:
        """逐項 PARAM_REQUEST_READ → PARAM_VALUE(mavlink_link.py:1649-1657 的做法)。

        ⚠️ 讀不到 = None,呼叫端要把它當**違規**不是跳過。
           「沒讀到所以算過」正是這一整類 bug 的形狀(contract_verify.py 檔頭)。
        """
        out: dict[str, dict[str, float | None]] = {l.key: {} for l in links}
        for name in names:
            def _send(l: Link, n: str = name) -> None:
                l.conn.mav.param_request_read_send(*l.target,
                                                   n.encode("ascii"), -1)
            msgs = self.gather(
                links, _send, ("PARAM_VALUE",),
                lambda l, n=name: (lambda m: _param_id(m) == n),
                float(self.cfg.cmd_timeout), int(self.cfg.cmd_retries))
            for k, m in msgs.items():
                out[k][name] = None if m is None else float(m.param_value)
        return out

    def set_params(self, links: Sequence[Link], params: dict[str, float]
                   ) -> dict[str, dict[str, bool]]:
        """PARAM_SET → **以 PARAM_VALUE 讀回比對**(mavlink_link.py:1632-1647)。

        ⚠️ 「我有送」不是證據。飛控可以夾住、可以拒收、可以改名後靜默略過
           (memory: sitl-param-renames-silent —— ArduPilot 4.7/4.8 對 --defaults 裡
           未知的參數名是**靜默略過**)。只有讀回值算數。
        """
        tol = float(self.cfg.param_tol)
        out: dict[str, dict[str, bool]] = {l.key: {} for l in links}
        for name, value in params.items():
            def _send(l: Link, n: str = name, v: float = float(value)) -> None:
                l.conn.mav.param_set_send(*l.target, n.encode("ascii"),
                                          float(v), _mav.MAV_PARAM_TYPE_REAL32)
            msgs = self.gather(
                links, _send, ("PARAM_VALUE",),
                lambda l, n=name: (lambda m: _param_id(m) == n),
                float(self.cfg.cmd_timeout), int(self.cfg.cmd_retries))
            for k, m in msgs.items():
                if m is None:
                    out[k][name] = False
                else:
                    got = float(m.param_value)
                    out[k][name] = abs(got - float(value)) <= tol * max(1.0, abs(float(value)))
        return out

    def set_mode(self, links: Sequence[Link], mode_of: Callable[[Link], str]
                 ) -> dict[str, tuple[bool, str]]:
        """切模式並**用 HEARTBEAT.custom_mode 讀回確認**(mavlink_link.py:1672-1687)。

        ⚠️ 兩段都要:ACK 只證明「指令被收下」,心跳的 custom_mode 才證明「真的切過去了」。
           模式編號**逐機**查(見 Link.mode_by_number 的說明)。
        """
        want: dict[str, int] = {}
        targets: list[Link] = []
        result: dict[str, tuple[bool, str]] = {}
        for l in links:
            table = l.mode_by_name()
            name = mode_of(l).upper()
            if name not in table:
                result[l.key] = (False, f"該機體的模式表裡沒有 {name}(可用:{sorted(table)[:8]}…)")
                continue
            want[l.key] = table[name]
            targets.append(l)

        def _send(l: Link) -> None:
            l.conn.mav.command_long_send(
                *l.target, _mav.MAV_CMD_DO_SET_MODE, 0,
                float(_mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(want[l.key]),
                0.0, 0.0, 0.0, 0.0, 0.0)

        acks = self.gather(targets, _send, ("COMMAND_ACK",),
                           lambda l: (lambda m: int(m.command) == _mav.MAV_CMD_DO_SET_MODE),
                           float(self.cfg.cmd_timeout), int(self.cfg.cmd_retries))
        # 讀回:等心跳的 custom_mode 真的變成我們要的那個
        done = self.wait_until(targets, lambda l: (l.hb_msg is not None
                                                   and int(l.hb_msg.custom_mode) == want[l.key]),
                               float(self.cfg.cmd_timeout) * 2)
        for l in targets:
            ok = bool(done.get(l.key))
            res = _result_name(acks.get(l.key))
            result[l.key] = (ok, f"ACK={res} 讀回模式={l.mode_name}"
                                 + ("" if ok else "  ← ⚠️ 心跳沒有變成目標模式"))
        return result


def _param_id(msg: Any) -> str:
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="replace")
    return str(pid).rstrip("\x00")


# ══════════════════════════════════════════════════════════════════════
# 六、逐機報表
# ══════════════════════════════════════════════════════════════════════
class Report:
    """逐機、逐步驟的成敗紀錄。

    ⚠️ 要求是「每一步都**逐機**回報成功或失敗,不要只報總結」。
       只報總結的話,16 架裡壞掉 1 架與壞掉 16 架長得一樣。
    """

    def __init__(self) -> None:
        self.steps: list[dict[str, Any]] = []
        self.failed = False

    def add(self, step: str, rows: list[tuple[str, bool, str]], note: str = "") -> bool:
        ok_all = all(ok for _, ok, _ in rows) and bool(rows)
        self.steps.append({"step": step, "ok": ok_all, "note": note,
                           "rows": [{"uav": u, "ok": o, "detail": d} for u, o, d in rows]})
        print(f"\n── {step} " + "─" * max(4, 62 - len(step)))
        if note:
            print(f"   {note}")
        for uav, ok, detail in rows:
            print(f"   {'✔' if ok else '✗'} {uav:<8} {detail}")
        if not rows:
            print("   ✗ 沒有任何機體 —— 這本身就是失敗")
            ok_all = False
        if not ok_all:
            self.failed = True
        return ok_all

    def dump(self, path: str | None, cfg: Config, extra: dict[str, Any] | None = None) -> None:
        blob = {"t_epoch": time.time(), "config": cfg.to_dict(), "steps": self.steps,
                "ok": not self.failed}
        if extra:
            blob.update(extra)
        text = json.dumps(blob, ensure_ascii=False, indent=2)
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text + "\n")
            print(f"\n[report] 已寫出 {path}")
        else:
            print(text)


# ══════════════════════════════════════════════════════════════════════
# 七、步驟
# ══════════════════════════════════════════════════════════════════════
def step_connect(fleet: Fleet, rep: Report, cfg: Config) -> bool:
    """連線 → 等心跳 → **從 srcSystem 讀回 sysid**。"""
    rows: list[tuple[str, bool, str]] = []
    for l in fleet.links:
        err = l.connect()
        if err is None:
            fleet.register(l)
    deadline = time.monotonic() + float(cfg.connect_timeout)
    while time.monotonic() < deadline:
        fleet.pump(0.2)
        if all(l.sysid is not None or l.dead for l in fleet.links):
            break
    for l in fleet.links:
        if l.dead:
            rows.append((l.key, False, f"{l.conn_str} — {l.dead_reason}"))
            continue
        if l.sysid is None:
            # ⚠️⚠️ 沒有心跳 ⇒ **一定要判死**,不能只印一行紅字放它繼續走下去:
            #    定不到址的機體如果繼續參與後面的步驟,唯一「能動」的走法就是把
            #    target_system 填 0,而那是**廣播** —— 指令會打到同鏈路上的所有機體。
            #    ⇒ 這裡判死是第一道;`Link.target` 的例外是第二道(萬一有人漏了這裡)。
            #      兩道都要,因為單靠「呼叫順序正確」的保護,在重構時是第一個消失的東西。
            l._die("連上了但沒有心跳 ⇒ 無法取得 sysid,不可定址")
            fleet.retire(l)
            rows.append((l.key, False,
                         f"{l.conn_str} 連上了但 {cfg.connect_timeout:.0f} s 內沒有心跳"
                         f"(已判死,不會對它送任何指令)。"
                         f"⚠️ SERIAL0 帶 :wait 時 SITL 卡在 accept()(UARTDriver.cpp:415-424),"
                         f"看起來就是這樣;也可能是這一架的 lock_step 從沒收到第一個 FDM 封包 ⇒ "
                         f"ArduPilotPlugin.cc:1204-1213 的迴圈立刻退出 ⇒ "
                         f"**world 照跑,它靜靜死掉**(事實【3】)"))
            continue
        type_name = _mav.enums["MAV_TYPE"].get(l.hb_type)
        detail = (f"{l.conn_str}  sysid={l.sysid} comp={l.compid}  "
                  f"MAV_TYPE={type_name.name if type_name else l.hb_type}  "
                  f"profile(宣告)={l.profile}  mode={l.mode_name}")
        row_ok = True

        # ── ⚠️⚠️ 韌體族系 vs 宣告的 profile ────────────────────────────
        # HEARTBEAT.type 分不出 VTOL 與純固定翼(quadplane.cpp:4042-4048,Q_MAV_TYPE=0
        # 時一律回 MAV_TYPE_FIXED_WING)—— 那一段結論不變。但它**分得出**
        # plane 韌體與 copter 韌體,而那正是 Link.mode_by_number() 用來挑模式表的東西。
        # 宣告錯了不擋下來的話,後面每一個模式名都是「看起來很合理但完全錯誤」的,
        # 而模式讀回是本檔證明「切模式真的生效」的唯一手段 ⇒ 錯的表 = 假綠燈。
        fam = _fw_family(l)
        want_fam = "copter" if l.profile == "copter" else "plane"
        if fam is not None and fam != want_fam:
            row_ok = False
            detail += (f"\n        ✗ ⚠️⚠️ 心跳說這是 **{fam}** 韌體,但宣告的 profile "
                       f"`{l.profile}` 屬於 {want_fam} 家族 ⇒ 多半是埠算術把你接到別人的"
                       f"飛機上了(檔頭 §2.5),或 --instances 的 profile 打錯。"
                       f"不擋下來的話模式表會整組挑錯"
                       f"(mavutil.py:2383-2419 AP_MAV_TYPE_MODE_MAP)")

        # ── ⚠️⚠️ 同鏈路混入第二個 sysid ────────────────────────────────
        if l.foreign_sysids:
            if bool(cfg.allow_foreign_sysid):
                detail += (f"\n        ⚠️ 同鏈路還看到別的 sysid {sorted(l.foreign_sysids)}"
                           f"(--allow-foreign-sysid 已放行)")
            else:
                row_ok = False
                detail += (f"\n        ✗ ⚠️⚠️ 同鏈路還看到別的 sysid "
                           f"{sorted(l.foreign_sysids)} —— 一條直連 SITL SERIAL 的 TCP "
                           f"只會有一個 sysid。出現第二個代表這是 MAVProxy / fanout 的"
                           f"共用埠 ⇒ **我們不知道自己在對誰下令**。事實【2】:埠被偷走在 "
                           f"`ss` 那一層完全隱形,這是 MAVLink 層唯一看得見的影子。"
                           f"真的要接共用埠請加 --allow-foreign-sysid")
        rows.append((l.key, row_ok, detail))

    # ── ⚠️⚠️⚠️ 跨鏈路:sysid 必須兩兩相異 ──────────────────────────────
    # 這是本檔唯一能證明「我們接到的是 N 架**不同**的飛機」的檢查。
    # 少了它,兩條鏈路指到同一架(埠算術錯位、from-state 埠重複、或某個埠其實是
    # 別人的 fanout)會**整份報表全綠** —— 每一步都成功,只是全部打在同一架身上。
    # sysid 讀回本身不構成這個證據:原版讀了、印了,但從來沒有拿它做過任何判斷。
    by_sysid: dict[int, list[str]] = {}
    for l in fleet.links:
        if l.sysid is not None:
            by_sysid.setdefault(int(l.sysid), []).append(f"{l.key}({l.conn_str})")
    dup_rows = [(",".join(keys), False,
                 f"⚠️⚠️⚠️ 這 {len(keys)} 條鏈路讀回**同一個** sysid={sid} ⇒ 它們接的是"
                 f"同一架飛機(或同一個 fanout 埠)。後面每一步都會成功,只是全部打在"
                 f"同一架身上 —— 這是整份報表全綠而任務其實沒發生的那個形狀。"
                 f"⇒ 用 --from-state 讀啟動器寫下的生效埠,不要自己算(檔頭 §2.5)")
                for sid, keys in sorted(by_sysid.items()) if len(keys) > 1]
    if dup_rows:
        rows += dup_rows
        for l in fleet.links:
            if l.sysid is not None and len(by_sysid[int(l.sysid)]) > 1:
                l._die(f"sysid {l.sysid} 與其他鏈路重複 ⇒ 不確定在對誰下令")
                fleet.retire(l)

    return rep.add("1. 連線與心跳(sysid 一律讀回,不假設等於索引;並檢查兩兩相異)", rows)


def _fw_family(l: Link) -> str | None:
    """由 HEARTBEAT.type 判「plane 韌體」還是「copter 韌體」。分不出來回 None。

    ⚠️ 只分得到這個粒度:VTOL 與純固定翼在 MAVLink 上一模一樣
       (quadplane.cpp:4042-4048)。要分那兩者只有 Q_ENABLE 讀回。

    ⚠️ 刻意**不**列舉 MAV_TYPE_* 常數名:本機 pymavlink 2.4.49 裡
       MAV_TYPE_VTOL_TAILSITTER / _FIXEDROTOR / _TILTWING 等新名字**不存在**
       (2026-09-09 實測),寫進去會在執行時炸 AttributeError,而且只在
       「剛好連到那種機體」時才炸 —— 那是最難重現的一種。
       改成直接問 pymavlink 自己挑了哪一張模式表(mavutil.py:2383-2419
       AP_MAV_TYPE_MODE_MAP),因為「挑錯表」正是我們要防的實害:
       同一個 custom_mode=15 在 apm 表是 GUIDED、在 acm 表是 AUTOTUNE
       (2026-09-09 實測)⇒ 錯的表會給出**看起來很合理**的錯誤模式名。
    """
    t = l.hb_type
    if t is None:
        return None
    table = _mavutil.mode_mapping_bynumber(int(t))
    if table is _mavutil.mode_mapping_apm:
        return "plane"
    if table is _mavutil.mode_mapping_acm:
        return "copter"
    return None


def step_streams(fleet: Fleet, rep: Report, cfg: Config) -> bool:
    """⚠️⚠️ 請求串流 → 讀回間隔 → **實測頻率** → 順便量逐機模擬時鐘。"""
    links = fleet.live
    if not links:
        return rep.add("2. 資料串流", [])

    fast_us = int(round(1e6 / max(0.01, float(cfg.stream_hz))))
    slow_us = int(round(1e6 / max(0.01, float(cfg.slow_stream_hz))))

    # (1) SET_MESSAGE_INTERVAL,逐則,有 ACK(GCS_Common.cpp:5891)
    set_ack: dict[str, dict[str, int | None]] = {l.key: {} for l in links}
    for msg_id, name, is_fast in STREAM_MSGS:
        res = fleet.command_long(links, _mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                                 [float(msg_id), float(fast_us if is_fast else slow_us)])
        for k, v in res.items():
            set_ack[k][name] = v

    # (2) 保底:REQUEST_DATA_STREAM。⚠️ 它**沒有 ACK**,所以它自己不構成任何證據 ——
    #     放在這裡只是為了讓沒有 511 的舊韌體也能動。真正的判準是下面的實測。
    for l in links:
        for sid in (_mav.MAV_DATA_STREAM_POSITION, _mav.MAV_DATA_STREAM_EXTRA1,
                    _mav.MAV_DATA_STREAM_EXTRA2, _mav.MAV_DATA_STREAM_EXTENDED_STATUS):
            try:
                l.conn.mav.request_data_stream_send(
                    *l.target, sid,
                    int(max(1, round(float(cfg.stream_hz)))), 1)
            except (OSError, AttributeError):
                pass

    # (3) 讀回間隔:GET_MESSAGE_INTERVAL → MESSAGE_INTERVAL(GCS_Common.cpp:3350-3376)
    #     回 -1 = disabled、0 = 這個訊息在這個韌體上不存在。
    readback: dict[str, dict[str, float | None]] = {l.key: {} for l in links}
    for msg_id, name, _is_fast in STREAM_MSGS:
        msgs = fleet.gather(
            links,
            lambda l, mid=msg_id: l.conn.mav.command_long_send(
                *l.target, _mav.MAV_CMD_GET_MESSAGE_INTERVAL, 0,
                float(mid), 0, 0, 0, 0, 0, 0),
            ("MESSAGE_INTERVAL",),
            lambda l, mid=msg_id: (lambda m: int(m.message_id) == int(mid)),
            float(cfg.cmd_timeout), int(cfg.cmd_retries))
        for k, m in msgs.items():
            readback[k][name] = None if m is None else float(m.interval_us)

    # (4) ⚠️ 唯一算數的一步:實測。同時記下 time_boot_ms 的起點。
    base_counts = {l.key: dict(l.counts) for l in links}
    base_boot = {l.key: (l.boot_ms, l.boot_ms_epoch) for l in links}
    t0 = time.monotonic()
    fleet.pump(float(cfg.rate_window))
    elapsed = max(1e-6, time.monotonic() - t0)

    t_end = time.monotonic()
    stale_budget = max(0.5, float(cfg.rate_window) * float(cfg.rtf_stale_frac))

    rows: list[tuple[str, bool, str]] = []
    for l in links:
        if l.dead:
            rows.append((l.key, False, f"鏈路死亡:{l.dead_reason}"))
            continue
        parts, ok = [], True

        # ── 逐機模擬時鐘。⚠️ 這一欄是分辨「那一架卡死」與「world 只是慢」的唯一工具,
        #    而且**要先算**,因為底下的頻率門檻要拿它做正規化。
        #
        #    ⚠️⚠️ 兩處與原版不同,兩處都是為了讓這個門檻**真的可能為假**:
        #    (1) 分母改成**整個視窗**(t_end − t0),不是「最後一次更新的時刻 − 起點」。
        #        原版的分母會隨著鏈路凍結而一起停住 ⇒ 視窗開頭收到一則 ATTITUDE、
        #        之後 5 秒全死,算出來是那 10 ms 的**瞬時**速率(可以是 20.0),
        #        整個視窗是死的卻亮綠燈。門檻放在一個不可能為假的地方 = 沒有門檻
        #        (memory: instrument-that-cannot-fire)。
        #    (2) 額外要求「最後一次更新」不能太舊。只看平均速率的話,前半段跑滿、
        #        後半段整段凍住,平均值仍可能過關。
        b0, e0 = base_boot[l.key]
        rtf: float | None = None
        if l.boot_ms is None or b0 is None or e0 is None or l.boot_ms_epoch is None:
            rtf_txt, rtf_ok = "sim_rtf 量不到(整個視窗沒有 ATTITUDE/位置訊息)", False
        else:
            rtf = (l.boot_ms - b0) / 1000.0 / elapsed
            stale = t_end - l.boot_ms_epoch
            fresh_ok = stale <= stale_budget
            rtf_ok = (rtf >= float(cfg.min_sim_rtf)) and fresh_ok
            rtf_txt = f"sim_rtf {rtf:.3f}(視窗 {elapsed:.1f}s,最後更新 {stale:.1f}s 前)"
            if not rtf_ok:
                rtf_txt += (
                    " ✗ ⚠️⚠️ "
                    + ("模擬時鐘沒有前進" if rtf < float(cfg.min_sim_rtf)
                       else f"模擬時鐘**中途停了**(> {stale_budget:.1f}s 沒更新)")
                    + " ⇒ **這一架**的 lock_step 卡住了。"
                    "gz 的 /stats sim_time 這時候照樣會前進(整個 world 由其他機推),"
                    "所以只看 /stats 判別不出來(ArduPilotPlugin.cc:1204-1213)")
        ok &= rtf_ok

        # ⚠️⚠️ 串流是照**模擬時間**排的,所以牆鐘頻率 ≈ 要求頻率 × sim_rtf。
        #    原版直接拿牆鐘頻率去比 `要求 × 0.5`:lock_step RTF 0.236(事實【1】四機實測)
        #    ⇒ 要求 5 Hz 的訊息牆鐘只有 1.18 Hz < 2.5 ⇒ **健康機隊必紅**。
        #    紅字滿天飛的閘門會被人關掉,關掉之後真的壞的那次也不會紅。
        #    ⇒ 這裡把牆鐘頻率除以量到的 sim_rtf 再比;raw 值照樣印出來給人判讀。
        #    量不到 sim_rtf 時 norm 用 1.0(此時 rtf_ok 已經是 False,不會假綠)。
        norm = rtf if (rtf is not None and rtf > 1e-6) else 1.0

        hb_hz = (l.counts.get("HEARTBEAT", 0) - base_counts[l.key].get("HEARTBEAT", 0)) / elapsed
        hb_ok = (hb_hz / norm) >= float(cfg.hb_floor_hz)
        ok &= hb_ok
        parts.append(f"HEARTBEAT {hb_hz:.2f}Hz(÷rtf={hb_hz / norm:.2f}"
                     f"/門檻 {float(cfg.hb_floor_hz):.2f}){'' if hb_ok else ' ✗'}")

        for _msg_id, name, is_fast in STREAM_MSGS:
            want = float(cfg.stream_hz if is_fast else cfg.slow_stream_hz)
            got = (l.counts.get(name, 0) - base_counts[l.key].get(name, 0)) / elapsed
            floor = want * float(cfg.rate_floor_frac)
            good = (got / norm) >= floor
            ok &= good
            iv = readback[l.key].get(name)
            iv_txt = ("讀不到" if iv is None else
                      "disabled" if iv < 0 else
                      "不支援" if iv == 0 else f"{1e6 / iv:.1f}Hz")
            parts.append(f"{name} {got:.2f}Hz(÷rtf={got / norm:.2f}/門檻 {floor:.1f})"
                         f"(讀回 {iv_txt})" + ("" if good else " ✗"))
            if not good and set_ack[l.key].get(name) not in (None, _mav.MAV_RESULT_ACCEPTED):
                parts[-1] += f"[SET_MESSAGE_INTERVAL {_result_name(set_ack[l.key].get(name))}]"

        parts.append(rtf_txt)

        # ⚠️⚠️ 混入第二個 sysid **要判紅**,不能只在細節字串裡加一行 ⚠️ 然後那一列照樣 ✔。
        #    一條直連 SITL SERIAL 的 TCP 只會有一個 sysid;出現第二個代表這個埠是
        #    MAVProxy / fanout 的共用埠 ⇒ 我們不知道自己在對誰下令。
        #    事實【2】:埠被偷走在 ss 那一層是隱形的,這是 MAVLink 層唯一看得見的影子。
        if l.foreign_sysids:
            foreign_ok = bool(cfg.allow_foreign_sysid)
            ok &= foreign_ok
            parts.append(f"{'⚠️' if foreign_ok else '✗ ⚠️⚠️'} 同鏈路混入 sysid "
                         f"{sorted(l.foreign_sysids)}"
                         + ("(--allow-foreign-sysid 已放行)" if foreign_ok else
                            " ⇒ 這不是某一架 SITL 的專屬 SERIAL 埠"))
        rows.append((l.key, bool(ok), "  ".join(parts)))

    note = (f"要求 快訊息 {cfg.stream_hz} Hz / 慢訊息 {cfg.slow_stream_hz} Hz;"
            f"實測門檻 = 要求 × {cfg.rate_floor_frac},比的是**牆鐘頻率 ÷ 量到的 sim_rtf**"
            f"(串流照模擬時間排,lock_step RTF 0.236 時牆鐘頻率本來就只有要求值的 1/4);"
            f"視窗 {cfg.rate_window} s、time_boot_ms 允許最舊 {stale_budget:.1f} s。"
            f"⚠️ ArduPlane 預設 POSITION/EXTRA1 只有 1 Hz、ArduCopter **全部 0** "
            f"(GCS_MAVLink_Parameters.cpp:49-71)⇒ 這一步一定要做,不能靠預設。"
            f"⚠️ 串流速率參數已改名為 MAV0_/MAV1_/MAV2_*,舊的 SR0_* 會被靜默略過")
    return rep.add("2. 資料串流:請求 → 讀回間隔 → **實測頻率** + 逐機模擬時鐘", rows, note)


def step_params(fleet: Fleet, rep: Report, cfg: Config,
                extra_by_scope: dict[str, dict[str, tuple[float | None, float | None]]]
                ) -> bool:
    """逐機參數契約:**讀回**比對。⚠️ 讀不到 = 違規,不是跳過。"""
    links = fleet.live
    if not links:
        return rep.add("3. 參數契約", [])
    if not bool(cfg.param_check):
        print("\n⚠️⚠️ --no-param-check:參數契約整段跳過。"
              "這是降級,而且它現在被大聲印出來了 —— 靜默降級是這一整類 bug 的成因。")
        return True

    # profile → 這一輪要讀的參數名(同 profile 的機體共用一次 gather)
    by_profile: dict[str, list[Link]] = {}
    for l in links:
        by_profile.setdefault(l.profile, []).append(l)

    rows: list[tuple[str, bool, str]] = []
    for profile, plinks in by_profile.items():
        table = list(PROFILES.get(profile, ()))
        extra = expect_for(extra_by_scope, profile)
        for name, (lo, hi) in extra.items():
            table = [c for c in table if c.name != name]
            table.append(ParamContract(name, lo, hi, None,
                                       "由 --expect 於命令列加入", "命令列"))
        # ⚠️⚠️ PWM 那兩條的期望值是**逐機**的(那一架自己 SDF 的 servo_min/servo_max),
        #    不是 profile 級的常數 —— 見 PWM_CONTRACT_NAMES 上面那段。--expect 明講的優先。
        pwm_names = tuple(n for n in PWM_CONTRACT_NAMES.get(profile, ())
                          if n not in extra)
        got = fleet.get_params(plinks, [c.name for c in table])
        for l in plinks:
            vals = got.get(l.key, {})
            bad: list[str] = []
            for c in table:
                if c.name in pwm_names:
                    if l.pwm_expect is None:
                        # ⚠️ 「沒有權威來源就跳過」= 靜默降級。這裡判違規並講清楚怎麼補。
                        bad.append(
                            f"{c.name}={'讀不到' if vals.get(c.name) is None else format(vals[c.name], 'g')}"
                            f" **未驗證**:期望值必須來自那一架 SDF 的 servo_min/servo_max,"
                            f"而本次沒有 ports.json({l.pwm_src})。"
                            f"⇒ 加 --ports-json <world>.ports.json(--from-state 會自動找),"
                            f"或用 --expect {c.name}=<值> 明講。"
                            f"⚠️ 不可以拿常數猜:gz_vtol_world.py 的 LIFT_PWM_*_DEFAULT "
                            f"本身就能被 GZ_VTOL_LIFT_PWM_MIN/MAX 覆寫,而上游 iris 的 SDF "
                            f"是 1100/1900 —— 猜 1000/2000 會在真的對不上時亮綠燈｜{c.evidence}")
                        continue
                    want = l.pwm_expect[0] if c.name.endswith("_MIN") else l.pwm_expect[1]
                    c = ParamContract(c.name, want, want, None,
                                      f"必須等於這一架 SDF 升力通道的 "
                                      f"servo_{'min' if c.name.endswith('_MIN') else 'max'}"
                                      f" = {want:g}({l.pwm_src})", c.evidence)
                v = vals.get(c.name)
                if v is None:
                    # ⚠️ contract_verify.py 檔頭:「沒讀到所以算過」正是這一類 bug 的形狀。
                    bad.append(f"{c.name}=讀不到(⇒ 違規,不是跳過)｜{c.why}｜證據:{c.evidence}")
                    continue
                if c.fallback is not None and abs(v - float(c.fallback)) <= 1e-9:
                    bad.append(f"{c.name}={v:g} 命中**退回值**(= 沒有人設過它)｜{c.why}"
                               f"｜證據:{c.evidence}")
                    continue
                if (c.lo is not None and v < c.lo - 1e-9) or (c.hi is not None and v > c.hi + 1e-9):
                    bad.append(f"{c.name}={v:g} 落在 [{c.lo},{c.hi}] 外｜{c.why}"
                               f"｜證據:{c.evidence}")
            head = f"profile={profile} 檢查 {len(table)} 項"
            if bad:
                rows.append((l.key, False, head + "\n        - " + "\n        - ".join(bad)))
            else:
                rows.append((l.key, True, head + " 全部通過"))
    note = ("⚠️ profile 是**宣告**,這一步用讀回值打臉宣告:"
            "宣告 vtol 而 Q_ENABLE 讀不到或為 0 ⇒ 這裡就會紅。"
            "HEARTBEAT.type 判別不出 VTOL(quadplane.cpp:4042-4048 預設回 MAV_TYPE_FIXED_WING)")
    return rep.add("3. 逐機參數契約(讀回比對;讀不到 = 違規)", rows, note)


def step_origin(fleet: Fleet, rep: Report, cfg: Config) -> bool:
    """HOME 與 world 的 spherical_coordinates 原點是否對得上。"""
    links = fleet.live
    if not links:
        return rep.add("4. HOME 與 world 原點", [])
    fleet.gather(
        links,
        lambda l: l.conn.mav.command_long_send(
            *l.target, _mav.MAV_CMD_REQUEST_MESSAGE, 0,
            float(_mav.MAVLINK_MSG_ID_HOME_POSITION), 0, 0, 0, 0, 0, 0),
        ("HOME_POSITION",), None, float(cfg.cmd_timeout), int(cfg.cmd_retries))
    fleet.pump(0.5)

    rows: list[tuple[str, bool, str]] = []
    olat, olon, oalt = float(cfg.origin_lat), float(cfg.origin_lon), float(cfg.origin_alt)
    for l in links:
        hp = l.latest.get("HOME_POSITION")
        if hp is None:
            rows.append((l.key, False,
                         "拿不到 HOME_POSITION ⇒ 無法確認 --home 與 world 原點一致"))
            continue
        hlat, hlon, halt = hp.latitude / 1e7, hp.longitude / 1e7, hp.altitude / 1000.0
        dy = (hlat - olat) * 111_320.0
        dx = (hlon - olon) * 111_320.0 * math.cos(math.radians(olat))
        d = math.hypot(dx, dy)
        dz = halt - oalt
        ok = d <= float(cfg.origin_tol_m) and abs(dz) <= float(cfg.origin_alt_tol_m)
        rows.append((l.key, ok,
                     f"HOME {hlat:.6f},{hlon:.6f},{halt:.1f}m  vs 原點 {olat},{olon},{oalt}  "
                     f"水平 {d:.1f} m / 高度 {dz:+.1f} m"
                     + ("" if ok else "  ← ⚠️ SITL 的 --home 與 world 的 "
                                      "<spherical_coordinates> 不一致 ⇒ 座標基準整個差開")))
    note = (f"world 原點取自 taoyuan_runway.sdf:50-56(可用 --origin-* 覆寫)。"
            f"⚠️ 該檔未解問題:地面在世界 z=0 而 elevation 寫 {oalt:g} "
            f"⇒ Gazebo 的『海平面』其實是 {oalt:g} m AMSL,所以高度容差刻意放寬到 "
            f"{cfg.origin_alt_tol_m:g} m")
    return rep.add("4. HOME 與 world 原點對位", rows, note)


def step_arm(fleet: Fleet, rep: Report, cfg: Config, dry: bool) -> bool:
    """切 GUIDED → 解鎖 → **讀回 SAFETY_ARMED**。"""
    links = fleet.live
    if not links:
        return rep.add("5. 解鎖", [])
    if dry:
        return rep.add("5. 解鎖(--dry-run 跳過)",
                       [(l.key, True, "dry-run:不送 ARM") for l in links])

    mode_res = fleet.set_mode(links, lambda l: PROFILE_MODES.get(l.profile, ("GUIDED",))[0])
    mode_rows = [(l.key, mode_res.get(l.key, (False, "沒跑到"))[0],
                  mode_res.get(l.key, (False, "沒跑到"))[1]) for l in links]
    ok_mode = rep.add("5a. 切 GUIDED(ACK + HEARTBEAT.custom_mode 讀回)", mode_rows,
                      "⚠️ quadplane.cpp:4004-4007 do_user_takeoff 的第一道檢查就是"
                      "『control_mode == guided』,不在 GUIDED 起飛指令一定 MAV_RESULT_FAILED")

    armable = [l for l in links if mode_res.get(l.key, (False, ""))[0]]
    # ⚠️ param2 = 0:**絕不 force**。force(21196)會繞過 PreArm,把「組態有問題」變成
    #    「起飛後才發現」。memory: arming-require-kills-quadplane 那一趟全程 0 次 force。
    res = fleet.command_long(armable, _mav.MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0],
                             timeout_s=float(cfg.cmd_timeout) * 2)
    done = fleet.wait_until(armable, lambda l: l.armed, float(cfg.cmd_timeout) * 3)
    rows = []
    for l in links:
        if l not in armable:
            rows.append((l.key, False, "沒切到 GUIDED,不送 ARM"))
            continue
        ok = bool(done.get(l.key))
        detail = f"ACK={_result_name(res.get(l.key))} 讀回 armed={l.armed}"
        if not ok:
            # PreArm 的理由只在 STATUSTEXT 裡,不印出來就等於沒有診斷。
            st = l.recent_statustext(6)
            detail += "\n        最近的 STATUSTEXT(PreArm 理由通常在這裡):\n        - " + \
                      ("\n        - ".join(st) if st else "(沒有)")
        rows.append((l.key, ok, detail))
    return ok_mode and rep.add("5b. 解鎖(讀回 base_mode 的 SAFETY_ARMED)", rows)


def step_takeoff(fleet: Fleet, rep: Report, cfg: Config, dry: bool, use_int: bool) -> bool:
    """VTOL 起飛 → 等到達目標高度。"""
    links = [l for l in fleet.live if l.armed] if not dry else fleet.live
    if not links:
        return rep.add("6. 起飛", [])
    alt = float(cfg.takeoff_alt)
    if dry:
        return rep.add("6. 起飛(--dry-run 跳過)",
                       [(l.key, True, f"dry-run:不送 NAV_TAKEOFF alt={alt}") for l in links])

    if use_int:
        # 顯式 COMMAND_INT:frame 與 z 的正負號**逐 profile**不同(§4.2)。
        rows_cmd: list[tuple[str, bool, str]] = []
        for profile, plinks in _group(links).items():
            frame, sign = PROFILE_TAKEOFF_INT.get(profile, PROFILE_TAKEOFF_INT["vtol"])
            res = fleet.command_int(plinks, _mav.MAV_CMD_NAV_TAKEOFF, frame,
                                    [0.0, 0.0, 0.0, 0.0], 0.0, 0.0, sign * alt)
            fname = _mav.enums["MAV_FRAME"].get(frame)
            for l in plinks:
                r = res.get(l.key)
                rows_cmd.append((l.key, r == _mav.MAV_RESULT_ACCEPTED,
                                 f"COMMAND_INT NAV_TAKEOFF frame="
                                 f"{fname.name if fname else frame} z={sign * alt:+.1f} "
                                 f"→ {_result_name(r)}"))
        note = ("--takeoff-command-int:frame/正負號由本檔按 profile 填。"
                "⚠️ Plane 只收 MAV_FRAME_LOCAL_OFFSET_NED 且 z 取負"
                "(GCS_MAVLink_Plane.cpp:892-897);Copter 只收 MAV_FRAME_GLOBAL_RELATIVE_ALT"
                "(=3,**不是** _INT=6)且 z 取正(GCS_MAVLink_Copter.cpp:580-590)。填錯是 DENIED")
    else:
        # 預設:COMMAND_LONG param7 = 高度(公尺,向上為正),讓各家韌體自己轉 frame。
        res = fleet.command_long(links, _mav.MAV_CMD_NAV_TAKEOFF,
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt])
        rows_cmd = [(l.key, res.get(l.key) == _mav.MAV_RESULT_ACCEPTED,
                     f"COMMAND_LONG NAV_TAKEOFF param7={alt:.1f} → {_result_name(res.get(l.key))}"
                     + ("" if res.get(l.key) == _mav.MAV_RESULT_ACCEPTED else
                        "  ← ⚠️ DENIED 通常是 frame、FAILED 通常是"
                        " 不在 GUIDED / 沒解鎖 / 已經在飛(quadplane.cpp:4004-4015),"
                        " UNSUPPORTED 表示這個韌體根本不收這個指令"))
                    for l in links]
        note = ("預設走 COMMAND_LONG,讓**各家韌體自己**做 frame 轉換 —— "
                "會分岔的正是 frame 與正負號(Plane GCS_MAVLink_Plane.cpp:859-877 轉成 "
                "LOCAL_OFFSET_NED 且 z=-param7;Copter GCS_MAVLink_Copter.cpp:600-608 轉成 "
                "GLOBAL_RELATIVE_ALT),而每家韌體最知道自己那一套。"
                "⚠️ ArduPlane **不 dispatch** MAV_CMD_NAV_VTOL_TAKEOFF"
                "(GCS_MAVLink_Plane.cpp:781-782 只有 NAV_TAKEOFF)⇒ 送了會 UNSUPPORTED")
    ok_cmd = rep.add(f"6a. 起飛指令(目標 {alt:.1f} m,相對 HOME)", rows_cmd, note)

    climbing = [l for l in links if not l.dead]
    target = alt * float(cfg.takeoff_frac)

    def _tick(remaining: float) -> None:
        cur = " ".join(f"{l.key}:{(l.rel_alt_m or 0):.1f}m" for l in climbing)
        print(f"   … 爬升中(剩 {remaining:.0f}s):{cur}")

    done = fleet.wait_until(climbing, lambda l: (l.rel_alt_m or -1e9) >= target,
                            float(cfg.takeoff_timeout), tick=_tick)
    rows = [(l.key, bool(done.get(l.key)),
             f"relative_alt={(l.rel_alt_m if l.rel_alt_m is not None else float('nan')):.1f} m "
             f"(門檻 {target:.1f}) mode={l.mode_name} armed={l.armed}"
             + ("" if done.get(l.key) else
                "  ← 沒到位。⚠️ 若 sim_rtf 那一欄也是 0,先查 lock_step 不要查控制律"))
            for l in climbing]
    note = (f"⚠️ 高度一律用 GLOBAL_POSITION_INT.relative_alt(相對 HOME),不用 AMSL —— "
            f"只量差量,整類 datum 麻煩一起消掉。逾時 {cfg.takeoff_timeout:g} s 是**牆鐘**,"
            f"lock_step 下 RTF 可能只有 0.2 左右(事實【1】四機 0.236),要留餘裕")
    return ok_cmd and rep.add("6b. 到達目標高度", rows, note)


def step_land(fleet: Fleet, rep: Report, cfg: Config, dry: bool) -> bool:
    """降落:切 QLAND / LAND 並讀回,再等上鎖。"""
    links = fleet.live
    if not links:
        return rep.add("7. 降落", [])
    if dry:
        return rep.add("7. 降落(--dry-run 跳過)",
                       [(l.key, True, "dry-run:不送降落") for l in links])

    res = fleet.set_mode(links, lambda l: PROFILE_MODES.get(l.profile, ("GUIDED", "RTL"))[1])
    rows_mode = [(l.key, res.get(l.key, (False, "沒跑到"))[0],
                  res.get(l.key, (False, "沒跑到"))[1]) for l in links]
    note = ("⚠️ 降落**只能**走切模式:ArduPlane 的 handle_command_int_packet"
            "(GCS_MAVLink_Plane.cpp:731-801)裡**沒有** NAV_VTOL_LAND 也沒有 NAV_LAND "
            "⇒ 送指令是打空氣(會拿到 UNSUPPORTED)。"
            "而切模式有 HEARTBEAT.custom_mode 可以讀回,本來就比較硬")
    ok_mode = rep.add("7a. 切降落模式(讀回 custom_mode)", rows_mode, note)

    landing = [l for l in links if res.get(l.key, (False, ""))[0]]

    def _tick(remaining: float) -> None:
        cur = " ".join(f"{l.key}:{(l.rel_alt_m or 0):.1f}m/{'armed' if l.armed else 'disarmed'}"
                       for l in landing)
        print(f"   … 降落中(剩 {remaining:.0f}s):{cur}")

    done = fleet.wait_until(landing, lambda l: not l.armed, float(cfg.land_timeout), tick=_tick)
    rows = [(l.key, bool(done.get(l.key)),
             f"mode={l.mode_name} armed={l.armed} "
             f"relative_alt={(l.rel_alt_m if l.rel_alt_m is not None else float('nan')):.1f} m")
            for l in landing]
    for l in links:
        if l not in landing:
            rows.append((l.key, False, "沒切到降落模式"))
    return ok_mode and rep.add("7b. 落地上鎖(讀回 SAFETY_ARMED 清掉)", rows)


def cmd_goto(fleet: Fleet, rep: Report, cfg: Config, lat: float, lon: float, alt: float,
             radius: float, change_mode: bool, yaw_deg: float | None) -> bool:
    """DO_REPOSITION(COMMAND_INT)+ 三道守衛。見 §4.4。"""
    # ── 守衛 (a):param3 絕對不可以是 0 ────────────────────────────
    # ⚠️ 這一條**排在最前面**,連「有沒有機體」都不先問。理由:它擋的是呼叫端打錯字,
    #    與機隊狀態無關。放在 `if not links: return` 後面的話,機隊是空的時候
    #    守衛就靜靜地不執行 —— 而「守衛在某些路徑上不會觸發」正是本專案
    #    memory: instrument-that-cannot-fire 那一整類問題的形狀。
    if not math.isfinite(radius) or radius <= 0.0:
        raise SystemExit(
            "✗ --loiter-radius 必須是 > 0 的有限值。\n"
            "  理由:GCS_MAVLink_Plane.cpp:600 的守衛是 `if (!isnan(param3) && param3 > 0)`。\n"
            "  0(或 NaN)不通過 ⇒ active_radius 維持 0 ⇒ 退回 |WP_LOITER_RAD| × eas2tas²\n"
            "  ≈ 2030 m ⇒ **飛機會忠實地繞著目標盤旋而不是飛過去**,而且不會有任何告警。\n"
            "  這個守衛擋在這裡而不是靠使用者記得,是因為 0 是最自然的『我不在乎』寫法。")

    links = fleet.live
    if not links:
        return rep.add("goto / DO_REPOSITION", [])

    # ── 守衛 (b):不帶 CHANGE_MODE 時,先讀回確認真的在 GUIDED ────
    if not change_mode:
        fleet.pump(1.0)
        not_guided = [l for l in links if l.mode_name != "GUIDED"]
        if not_guided:
            rows = [(l.key, False,
                     f"mode={l.mode_name} ≠ GUIDED ⇒ 不送。"
                     f"GCS_MAVLink_Plane.cpp:585-586 的條件是 "
                     f"`(param2 & CHANGE_MODE) || control_mode == guided`,"
                     f"兩者皆非就走 :606 `return MAV_RESULT_FAILED`。"
                     f"要嘛先切 GUIDED,要嘛加 --reposition-change-mode(但那正是被列為"
                     f"實測災難的行為:編隊層帶 CHANGE_MODE)")
                    for l in not_guided]
            rows += [(l.key, True, f"mode={l.mode_name} ✔") for l in links if l not in not_guided]
            return rep.add("goto:前置檢查(必須在 GUIDED)", rows)

    param2 = float(_mav.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) if change_mode else 0.0
    param4 = float("nan") if yaw_deg is None else float(yaw_deg)
    frame = _mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    res = fleet.command_int(links, _mav.MAV_CMD_DO_REPOSITION, frame,
                            [-1.0, param2, float(radius), param4], lat, lon, float(alt))
    rows = [(l.key, res.get(l.key) == _mav.MAV_RESULT_ACCEPTED,
             f"DO_REPOSITION → {_result_name(res.get(l.key))}  "
             f"(lat={lat} lon={lon} rel_alt={alt} param3={radius} param2={param2:g})"
             + ("" if res.get(l.key) == _mav.MAV_RESULT_ACCEPTED else
                "  ← ⚠️ FAILED 幾乎一定是『不在 GUIDED 且沒帶 CHANGE_MODE』"
                "(GCS_MAVLink_Plane.cpp:606)。這個訊息本來就會回來,"
                "以前之所以『零告警』是我們自己沒讀 COMMAND_ACK"))
            for l in links]
    note = ("⚠️ 用 COMMAND_INT 不用 COMMAND_LONG:x/y 是 int32 degE7,"
            "COMMAND_LONG 的 float32 裝不下 degE7 的 9 位有效數字 ⇒ 精度掉到公尺級。"
            "而且 ArduPlane 只在 handle_command_int_packet:734-735 處理 DO_REPOSITION。"
            "param1=-1 表示速度沿用預設")
    return rep.add("goto:DO_REPOSITION(COMMAND_INT + 讀 ACK)", rows, note)


def cmd_watch(fleet: Fleet, cfg: Config) -> None:
    """持續逐機印遙測。⚠️ 這是事實【3】要的那個『逐機讀 MAVLink 的姿態/位置在變』。"""
    print(f"\n── watch {cfg.watch_seconds:g} s(Ctrl-C 中止)"
          f"  ⚠️ 看的是**每一架自己**的 boot_ms 有沒有在動 ──")
    end = time.monotonic() + float(cfg.watch_seconds)
    prev = {l.key: (l.boot_ms, time.monotonic()) for l in fleet.links}
    while time.monotonic() < end:
        fleet.pump(float(cfg.watch_interval))
        cols = []
        for l in fleet.links:
            if l.dead:
                cols.append(f"{l.key}:DEAD")
                continue
            b0, t0 = prev.get(l.key, (None, time.monotonic()))
            now = time.monotonic()
            rtf = 0.0
            if l.boot_ms is not None and b0 is not None and now > t0:
                rtf = (l.boot_ms - b0) / 1000.0 / (now - t0)
            prev[l.key] = (l.boot_ms, now)
            att = l.latest.get("ATTITUDE")
            roll = math.degrees(att.roll) if att is not None else float("nan")
            pitch = math.degrees(att.pitch) if att is not None else float("nan")
            cols.append(f"{l.key}[{l.mode_name[:7]}{'A' if l.armed else '-'}] "
                        f"alt={(l.rel_alt_m if l.rel_alt_m is not None else float('nan')):6.1f} "
                        f"rp={roll:+5.1f}/{pitch:+5.1f} rtf={rtf:.3f}")
        print("  " + " | ".join(cols))


def _group(links: Sequence[Link]) -> dict[str, list[Link]]:
    out: dict[str, list[Link]] = {}
    for l in links:
        out.setdefault(l.profile, []).append(l)
    return out


# ══════════════════════════════════════════════════════════════════════
# 八、CLI
# ══════════════════════════════════════════════════════════════════════
#: gz_vtol_up.sh / gz_vtol_world.py 的 profile key → 本檔的 profile。
#: ⚠️ 兩邊的命名軸不同,而且**是刻意的**:那邊講的是「哪一個 Gazebo 模型目錄」
#:    (gz_vtol_world.py:258-320 的 PROFILES),本檔講的是「哪一套飛控參數契約」。
#:    對不上的 key 一律**大聲失敗**,不猜 —— 猜錯的話會拿 copter 的契約去驗 plane,
#:    而那種紅字看起來像「參數沒設好」,會把人帶去修根本沒壞的東西。
LAUNCHER_PROFILE_MAP: dict[str, str] = {
    "quadplane_vtol": "vtol",
    "iris": "copter",
    "zephyr": "plane",
}


#: ArduPilotPlugin 在 <control> 沒寫 servo_min/servo_max 時的**內建預設**。
#: 出處:gz_vtol_world.py:538-541 的同一組值(「plugin 的內建預設就是 1100/1900」)。
#: ⚠️ 這裡只在 ports.json 的那一欄是 null 時當作已知值用,並且會在報表裡標明它是推定的。
PLUGIN_DEFAULT_SERVO_MIN, PLUGIN_DEFAULT_SERVO_MAX = 1100.0, 1900.0


def validate_fleet_ports(links: Sequence[Link]) -> None:
    """⚠️⚠️ 兩條鏈路指到**同一個 TCP 埠**要在連線之前就擋下來。

    `parse_instances` 只擋了重複的 instance,擋不到重複的**埠** ——
    而 `@port` 與 `--from-state` 兩條路都是直接給埠,instance 各不相同、埠卻可能一樣。
    症狀:SITL 的 listen backlog 是 1,第二條要嘛連不上、要嘛接到別的東西,
    而報表上看起來只是「有一架比較慢」。
    (真的接到同一架的情況還有第二道:step_connect 的 sysid 兩兩相異檢查。)
    """
    by_port: dict[int, list[str]] = {}
    for l in links:
        by_port.setdefault(l.port, []).append(f"{l.key}[{l.port_src}]")
    dups = {p: v for p, v in by_port.items() if len(v) > 1}
    if dups:
        lines = "\n".join(f"    埠 {p}: {' / '.join(v)}" for p, v in sorted(dups.items()))
        raise SystemExit(
            "✗ 有多條鏈路算到**同一個 TCP 埠**:\n" + lines + "\n"
            "  ⇒ SITL 的 listen backlog 是 1,第二條會連不上或接到別的東西,\n"
            "    而那在報表上只會長成『有一架比較慢』。\n"
            "  ⇒ 用 --from-state 讀啟動器寫下的生效埠(檔頭 §2.5),不要自己算。")


def apply_ports_json(links: Sequence[Link], path: str, cfg: Config) -> None:
    """讀 `<world>.ports.json`(gz_vtol_world.py 產),把**逐機**的權威值接進來。

    ⚠️⚠️ 這是「兩邊讀同一份」那條規矩的落實(gz_vtol_world.py:1428-1429 明文要求)。
       在它之前,本檔把兩組常數抄在自己身上:
         (a) world 原點 25.0777 / 121.2325 / 32,而權威值在 ports.json 的 origin;
         (b) 升力馬達 PWM 刻度 1000/2000,而權威值是**那一架 SDF 自己的**
             servo_min/servo_max —— 上游 iris 是 1100/1900,而且 gz_vtol_world.py 的
             LIFT_PWM_*_DEFAULT 還能被 GZ_VTOL_LIFT_PWM_MIN/MAX 覆寫。
       抄過來的常數不會在對面改動時報錯,只會安靜地開始說謊。

    ⚠️ 對不上一律**硬失敗**,不退回猜測。
    """
    try:
        with open(path, encoding="utf-8") as fh:
            table = json.load(fh)
    except (OSError, ValueError) as e:
        raise SystemExit(f"✗ 讀不到/解不開 ports.json {path}:{e}")
    schema = table.get("schema")
    if schema != "aeroplan/gz_vtol_world/ports/1":
        raise SystemExit(
            f"✗ {path} 的 schema 是 {schema!r},不是 aeroplan/gz_vtol_world/ports/1。\n"
            f"  ⇒ 產生器換版了而本檔還在用舊欄位。這裡刻意不『盡量讀讀看』:\n"
            f"    欄位名對不上時 .get() 會回 None,而 None 會被下游當成『沒設過』——\n"
            f"    整份契約會安靜地退化成不檢查。")

    by_inst: dict[int, dict] = {}
    for v in table.get("vehicles", ()):
        try:
            by_inst[int(v["instance"])] = v
        except (KeyError, TypeError, ValueError):
            raise SystemExit(f"✗ {path} 的 vehicles 裡有一筆沒有可用的 instance:{v!r}")

    print(f"\n[ports] 權威對照表 {path}")
    print(f"        world={table.get('world_name')} md5={table.get('world_md5')} "
          f"partition={table.get('gz_partition')}")

    # ── 原點:ports.json 是權威。⚠️ 使用者在命令列/環境變數明講過的不覆寫,
    #    但要**印出來說兩邊不一致**,而不是安靜地選一邊。
    org = table.get("origin") or {}
    for knob_name, json_key in (("origin_lat", "latitude_deg"),
                                ("origin_lon", "longitude_deg"),
                                ("origin_alt", "elevation_m")):
        if json_key not in org:
            raise SystemExit(f"✗ {path} 的 origin 少了 {json_key} ⇒ 對照表不完整,不敢用")
        authoritative = float(org[json_key])
        src = cfg.sources.get(knob_name)
        if src == "default":
            cfg.values[knob_name] = authoritative
            cfg.sources[knob_name] = f"ports.json({path})"
        elif abs(float(cfg.values[knob_name]) - authoritative) > 1e-9:
            print(f"        ⚠️⚠️ {knob_name}:你指定了 {cfg.values[knob_name]}({src}),"
                  f"但 ports.json 的權威值是 {authoritative} —— 以你指定的為準,"
                  f"但這兩個不一致本身就值得先查清楚")
    print(f"        origin = {cfg.values['origin_lat']}, {cfg.values['origin_lon']}, "
          f"{cfg.values['origin_alt']}  [{cfg.sources.get('origin_lat')}]")

    for l in links:
        veh = by_inst.get(l.instance)
        if veh is None:
            raise SystemExit(
                f"✗ {path} 裡沒有 instance={l.instance} 這一架"
                f"(有的是 {sorted(by_inst)})。\n"
                f"  ⇒ 世界檔與 --instances 不是同一趟。這裡不猜:拿錯的 SDF 刻度去驗\n"
                f"    參數,紅字會把人帶去修根本沒壞的東西。")
        want_profile = LAUNCHER_PROFILE_MAP.get(str(veh.get("profile")))
        if want_profile is None:
            raise SystemExit(
                f"✗ {path} 的 I{l.instance} profile={veh.get('profile')!r} "
                f"對不到本檔的 profile;已知對照 {LAUNCHER_PROFILE_MAP}")
        if want_profile != l.profile:
            raise SystemExit(
                f"✗ I{l.instance}:ports.json 說它是 {veh.get('profile')!r}(⇒ {want_profile}),"
                f"但這一趟宣告的是 {l.profile!r}。\n"
                f"  ⇒ 用錯的參數契約去驗,紅字會指向沒壞的東西。先確認是哪一邊打錯。")

        # ── 升力通道的 PWM 刻度 ──────────────────────────────────────
        chans = {int(c["channel"]): c for c in (veh.get("channels") or ())}
        checked = [int(c) for c in (veh.get("lift_channels_checked") or ())]
        if not checked:
            # ⚠️ 純多旋翼(iris)在產生器那邊 lift_channels=() ⇒ 這一欄是空的,
            #    而它的四顆主升力就是 channel 0-3(SRV_Channel.h:82-85 motor1..4)。
            #    ⚠️ 對 QuadPlane 來說空 lift_channels_checked 是紅燈不是綠燈
            #      (gz_vtol_world.py:1511-1512 明文),所以只有 copter 走這個回退。
            if l.profile != "copter":
                raise SystemExit(
                    f"✗ I{l.instance}({l.profile})的 lift_channels_checked 是空的。\n"
                    f"  gz_vtol_world.py:1511-1512:『空 list = 沒有檢查任何東西,"
                    f"對 QuadPlane 來說那就是紅燈,不是綠燈』⇒ 世界檔那一側先修好再來。")
            checked = [0, 1, 2, 3]
        mins, maxs, guessed = set(), set(), False
        for ch in checked:
            c = chans.get(ch)
            if c is None:
                raise SystemExit(
                    f"✗ I{l.instance}:ports.json 說要看 channel {ch},但 channels 裡沒有它"
                    f"(有的是 {sorted(chans)})⇒ 對照表壞了")
            lo, hi = c.get("servo_min"), c.get("servo_max")
            if lo is None or hi is None:
                # SDF 省略時 plugin 用內建預設(gz_vtol_world.py:538-541)。
                lo = PLUGIN_DEFAULT_SERVO_MIN if lo is None else lo
                hi = PLUGIN_DEFAULT_SERVO_MAX if hi is None else hi
                guessed = True
            mins.add(float(lo))
            maxs.add(float(hi))
        if len(mins) != 1 or len(maxs) != 1:
            raise SystemExit(
                f"✗ I{l.instance} 的升力通道 {checked} PWM 刻度不一致"
                f"(min={sorted(mins)} max={sorted(maxs)})⇒ 同一個 Q_M_PWM_MIN/MAX "
                f"不可能同時對上兩組刻度,先修世界檔。")
        l.pwm_expect = (mins.pop(), maxs.pop())
        l.pwm_src = (f"{path} I{l.instance} channel {checked}"
                     + ("(SDF 省略 servo_min/max ⇒ 用 plugin 內建預設 1100/1900 推定)"
                        if guessed else ""))
        print(f"        I{l.instance:<4} {l.profile:<7} 升力通道 {checked} "
              f"servo {l.pwm_expect[0]:g}/{l.pwm_expect[1]:g}"
              + ("  ⚠️ 推定值" if guessed else ""))


def _parse_env_file(path: str) -> dict[str, str]:
    """讀 `KEY=value` / `KEY="value"` 形式的 state 檔。

    ⚠️ **不 source、不 exec**。那是一個 shell 檔,執行它等於把別的腳本的副作用
       (以及任何被塞進去的東西)帶進本行程。只做文字解析。
    """
    out: dict[str, str] = {}
    try:
        with open(path, encoding="utf-8") as fh:
            for raw in fh:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip()
                if len(v) >= 2 and v[0] == v[-1] and v[0] in ("\"", "'"):
                    v = v[1:-1]
                out[k] = v
    except OSError as e:
        raise SystemExit(f"✗ 讀不到 state 檔 {path}:{e}")
    return out


def instances_from_state(path: str) -> tuple[list[tuple[int, str, int | None]], dict[str, str]]:
    """從 `gz_vtol_up.sh` §10 寫下的 `vtol_swarm.env` 取**生效值**。

    這是最硬的一條路:`gz_vtol_world.py:1142-1143` 明文寫著「啟動器與 coordinator
    必須讀這一份,不要各自再算一次埠」。

    ⚠️ 任何一項對不上就**大聲失敗**,絕不悄悄退回算術。
       「讀不到就用預設」正是本專案要消滅的那個形狀(contract_verify.py 檔頭)。
    """
    env = _parse_env_file(path)
    insts_s = env.get("VTOL_INSTANCES", "").split()
    ports_s = env.get("VTOL_TCP_BASES", "").split()
    if not insts_s:
        raise SystemExit(f"✗ {path} 沒有 VTOL_INSTANCES ⇒ 這不是 gz_vtol_up.sh 的 state 檔,"
                         f"或那一趟沒跑到 §10 就死了。不要改用預設算術,先確認機隊真的起來了")
    if len(ports_s) != len(insts_s):
        raise SystemExit(f"✗ {path} 的 VTOL_INSTANCES({len(insts_s)} 筆)與 "
                         f"VTOL_TCP_BASES({len(ports_s)} 筆)數量不一致 ⇒ state 檔壞了,"
                         f"用 --instances <inst>@<port> 逐機明講")
    out: list[tuple[int, str, int | None]] = []
    for k, (i_s, p_s) in enumerate(zip(insts_s, ports_s)):
        try:
            inst, port = int(i_s), int(p_s)
        except ValueError:
            raise SystemExit(f"✗ {path} 第 {k} 架的 instance/port 不是整數:{i_s!r}/{p_s!r}")
        raw = env.get(f"VTOL_PROFILE_{k}", "")
        if not raw:
            raise SystemExit(f"✗ {path} 少了 VTOL_PROFILE_{k} ⇒ 無法決定要套哪一份參數契約。"
                             f"用 --instances {inst}:<profile>@{port} 明講")
        prof = LAUNCHER_PROFILE_MAP.get(raw)
        if prof is None:
            raise SystemExit(
                f"✗ {path} 的 VTOL_PROFILE_{k}={raw!r} 對不到本檔的 profile。\n"
                f"  已知對照:{LAUNCHER_PROFILE_MAP}\n"
                f"  ⇒ 啟動器加了新機種而本檔還不知道。請在 LAUNCHER_PROFILE_MAP 補一行,\n"
                f"    或這一趟先用 --instances {inst}:<vtol|copter|plane>@{port} 明講。\n"
                f"  ⚠️ 這裡刻意不猜:猜錯會拿錯的參數契約去驗,紅字會把人帶去修沒壞的東西。")
        out.append((inst, prof, port))
    print(f"[state] 由 {path} 讀回 {len(out)} 架(instance / profile / SERIAL0 基準埠):")
    for inst, prof, port in out:
        print(f"        I{inst:<4} {prof:<7} {port}")
    # ⚠️ 重複的 instance / 重複的埠在 state 檔裡同樣可能出現(啟動器算錯、或人手改過),
    #    而它們的症狀是「有一半的機體連對了」。這裡與 parse_instances 用同一組判準。
    seen_i: set[int] = set()
    for inst, _prof, _port in out:
        if inst in seen_i:
            raise SystemExit(f"✗ {path} 的 VTOL_INSTANCES 裡 instance {inst} 重複 ⇒ state 檔壞了")
        seen_i.add(inst)
    for extra in ("VTOL_RUN_ID", "GZ_VTOL_RUN_ID", "GZ_PARTITION", "VTOL_HOME",
                  "VTOL_SERIAL0_MODE", "VTOL_WORLD"):
        if extra in env:
            print(f"        {extra} = {env[extra]}")
    return out, env


def parse_instances(spec: str | None, default_profile: str) -> list[tuple[int, str, int | None]]:
    """`10,11,12` / `10:vtol,11:copter` / `10:vtol@6360` → [(instance, profile, port|None)]

    `@port` 給的是**那一架的 SERIAL0 基準埠**(--serial 的偏移仍會加上去),
    給了就完全繞過埠算術 —— 見檔頭 §2.5 的 ②。
    """
    if not spec:
        raise SystemExit(
            "✗ 必須給 --instances / GZ_VTOL_INSTANCES / --from-state 其中之一。\n"
            "  最建議:`--from-state <RUN>/vtol_swarm.env` —— 直接讀 gz_vtol_up.sh 寫下的\n"
            "          生效值,埠與 profile 都不用自己算(gz_vtol_world.py:1142-1143 明文\n"
            "          要求啟動器與 coordinator 讀同一份,不要各算各的)。\n"
            "  其次:`--instances 10:vtol@6360,11:vtol@6370`(逐機顯式埠)。\n"
            "  最後才是算術:`--instances 10,11,12` —— **只有 SITL 沒被給 --base-port 時才對**。\n"
            "  ⚠️ 這個參數**故意沒有預設值**:2026-09-06 實測另一個 session 的四機 Gazebo\n"
            "     佔著 TCP 5760…5793(-I0..-I3)與 UDP 9002/9012/9022/9032。\n"
            "     給一個『常用預設』等於埋一顆連到別人飛機上的地雷。\n"
            "     本線建議從 -I10 起(SITL TCP 6360+、JSON UDP 9102+、irlock 9105+)。")
    out: list[tuple[int, str, int | None]] = []
    seen: set[int] = set()
    for tok in _csv(spec):
        port: int | None = None
        if "@" in tok:
            tok, port_s = tok.split("@", 1)
            try:
                port = int(port_s)
            except ValueError:
                raise SystemExit(f"✗ --instances 的 @{port_s!r} 不是整數埠")
        if ":" in tok:
            inst_s, prof = tok.split(":", 1)
            prof = prof.strip().lower()
        else:
            inst_s, prof = tok, default_profile
        try:
            inst = int(inst_s)
        except ValueError:
            raise SystemExit(
                f"✗ --instances 的 {tok!r} 不是合法的 `<instance>[:<profile>][@<port>]`")
        if inst < 0:
            raise SystemExit(f"✗ instance 不可為負:{tok!r}")
        if prof not in PROFILES:
            raise SystemExit(f"✗ 未知 profile {prof!r};可用:{sorted(PROFILES)}")
        if inst in seen:
            raise SystemExit(f"✗ instance {inst} 重複。⚠️ 重複的 instance ⇒ 兩條 SITL 會算出"
                             f"同一組埠,而 SO_REUSEADDR 會讓 UDP 那一側**靜默**只餵其中一個"
                             f"(ArduPilotPlugin.cc:227 / SocketUDP.cc:72-75)")
        seen.add(inst)
        out.append((inst, prof, port))
    return out


def parse_expect(items: Iterable[str]
                 ) -> dict[str, dict[str, tuple[float | None, float | None]]]:
    """`--expect [profile:]NAME=lo[:hi]`(可重複)。只給 lo 表示要求剛好等於 lo。

    回 `{profile 或 "*": {NAME: (lo, hi)}}`。

    ⚠️⚠️ `profile:` 這個前綴是後補的,補的是一個**保證誤報**的行為:
       原版把每一條 --expect 無條件塞進**每一個** profile 的契約表,
       所以在混機種機隊上 `--expect MOT_PWM_MIN=1100`(只對 copter 有意義)
       會讓每一架 vtol 都多出一條「MOT_PWM_MIN 讀不到 ⇒ 違規」——
       而那個參數在 ArduPlane 上本來就不存在。
       紅字滿天飛的閘門會被人關掉,關掉之後真的壞的那次也不會紅
       (memory: verifying-the-verifier)。
       不寫前綴仍然是「套用到所有 profile」,維持舊行為。
    """
    out: dict[str, dict[str, tuple[float | None, float | None]]] = {}
    for it in items or ():
        if "=" not in it:
            raise SystemExit(f"✗ --expect 格式是 [profile:]NAME=lo[:hi],收到 {it!r}")
        name, rng = it.split("=", 1)
        scope = "*"
        if ":" in name:
            scope, name = name.split(":", 1)
            scope = scope.strip().lower()
            if scope not in PROFILES:
                raise SystemExit(f"✗ --expect 的 profile 前綴 {scope!r} 未知;"
                                 f"可用:{sorted(PROFILES)}")
        try:
            if ":" in rng:
                lo_s, hi_s = rng.split(":", 1)
                lo = None if lo_s.strip() == "" else float(lo_s)
                hi = None if hi_s.strip() == "" else float(hi_s)
            else:
                lo = hi = float(rng)
        except ValueError:
            raise SystemExit(f"✗ --expect {it!r} 的區間不是數字")
        out.setdefault(scope, {})[name.strip().upper()] = (lo, hi)
    return out


def expect_for(extra: dict[str, dict[str, tuple[float | None, float | None]]],
               profile: str) -> dict[str, tuple[float | None, float | None]]:
    """把 `*`(全體)與 `<profile>:`(指名)兩層合併,指名的優先。"""
    merged = dict(extra.get("*", {}))
    merged.update(extra.get(profile, {}))
    return merged


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="gz_vtol_coordinator.py",
        description="Gazebo + ArduPilot SITL 的異質 VTOL 群機 coordinator(pymavlink)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="每個旗標都有對應的環境變數,見 --dump-config 的輸出。")
    p.add_argument("mode", nargs="?", default="check",
                   choices=["check", "watch", "mission", "goto"],
                   help="check(預設,不會動螺旋槳)/ watch / mission(解鎖+起飛+降落)/ goto")
    # ⚠️ 所有 KNOB 的 default 一律留 None,好讓 resolve_config 分得出
    #    「使用者真的給了」與「argparse 塞了預設」——後者會蓋掉環境變數。
    for k in KNOBS:
        flag = "--" + k.cli.replace("_", "-")
        if k.cast is _b:
            p.add_argument(flag, dest=k.cli, default=None,
                           type=lambda s: _b(s), metavar="0|1", help=k.why)
            p.add_argument("--no-" + k.cli.replace("_", "-"), dest=k.cli,
                           action="store_const", const=False, help=argparse.SUPPRESS)
        else:
            p.add_argument(flag, dest=k.cli, default=None, type=k.cast, help=k.why)
    p.add_argument("--from-state", metavar="PATH",
                   help="讀 gz_vtol_up.sh §10 寫下的 vtol_swarm.env,取 instance / profile / "
                        "SERIAL0 埠的**生效值**(最建議的用法;與 --instances 互斥)")
    p.add_argument("--ports-json", metavar="PATH",
                   help="讀 gz_vtol_world.py 產的 `<world>.ports.json`,取 world 原點與"
                        "**逐機**升力通道的 servo_min/servo_max 當參數契約的期望值。"
                        "⚠️ 沒有它的話 Q_M_PWM_*/MOT_PWM_* 判『未驗證 ⇒ 違規』——"
                        "那兩個值不是常數,寫死會在真的對不上時亮綠燈(見 PWM_CONTRACT_NAMES)。"
                        "用 --from-state 時會自動由 VTOL_WORLD 推出來")
    p.add_argument("--expect", action="append", default=[], metavar="[profile:]NAME=lo[:hi]",
                   help="加一條參數契約(可重複),覆寫 profile 表裡的同名項。"
                        "⚠️ 不加 `profile:` 前綴就是**套用到所有 profile** —— "
                        "混機種機隊上 `--expect MOT_PWM_MIN=1100` 會讓每一架 vtol 都"
                        "多一條『讀不到 ⇒ 違規』(那參數在 ArduPlane 上不存在)。"
                        "指名寫法:`--expect copter:MOT_PWM_MIN=1100`")
    p.add_argument("--takeoff-command-int", action="store_true",
                   help="起飛改送顯式 COMMAND_INT(frame/正負號由本檔按 profile 填);"
                        "預設是 COMMAND_LONG 讓韌體自己轉,見檔頭 §4.2")
    p.add_argument("--dry-run", action="store_true",
                   help="mission 只做 check 的部分,不解鎖、不起飛、不降落")
    p.add_argument("--report", metavar="PATH",
                   help="把逐機逐步驟結果寫成 JSON;`-` = 印到 stdout。"
                        "不給就完全不輸出 JSON(人看的報表本來就已經印在上面了)")
    p.add_argument("--dump-config", action="store_true", help="印出生效值 JSON 後結束")
    # goto 專用
    p.add_argument("--lat", type=float, help="goto:目標緯度")
    p.add_argument("--lon", type=float, help="goto:目標經度")
    p.add_argument("--alt", type=float, help="goto:目標高度(公尺,相對 HOME)")
    p.add_argument("--loiter-radius", type=float, metavar="M",
                   help="goto:DO_REPOSITION 的 param3。⚠️ 必須 > 0,見檔頭 §4.4(a)")
    p.add_argument("--reposition-change-mode", action="store_true",
                   help="goto:param2 帶 MAV_DO_REPOSITION_FLAGS_CHANGE_MODE。"
                        "⚠️ 讓飛控替你切模式 —— 那正是被列為實測災難的行為,預設關閉")
    p.add_argument("--yaw", type=float, help="goto:param4 目標航向(度);不給 = NaN = 不指定")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    # ⚠️ --expect **在這裡**就解析,不要留到 step_params。原版寫成
    #    `step_params(..., parse_expect(args.expect))`,而 step_params 只在
    #    「至少有一架活著」時才會被呼叫 ⇒ 打錯的 --expect(未知 profile 前綴、
    #    區間不是數字)在全隊連不上的那一趟**完全不會被指出來**,
    #    而那正是你最需要它把每一件事都講清楚的一趟。
    #    守衛要放在「一定會執行到」的地方(memory: instrument-that-cannot-fire)。
    expect_table = parse_expect(args.expect)

    print("═" * 72)
    print(f"{CONFIG_SENTINEL}(cli > env > default;來源印在方括號裡)")
    print("═" * 72)
    for line in cfg.lines():
        print(line)
    if not bool(cfg.strict):
        print("\n⚠️⚠️ STRICT 已關閉:失敗只會警告不會以非 0 結束。"
              "這是降級,而且它現在被大聲印出來了。")
    if args.dump_config:
        print(json.dumps(cfg.to_dict(), ensure_ascii=False, indent=2))
        return 0

    if args.from_state:
        # ⚠️ 兩個真相來源同時存在,正是這條線上最貴的那一類 bug ——
        #    而且它的表徵是「有一半的機體連對了」,比全錯更難查。
        if cfg.instances:
            raise SystemExit(
                f"✗ --from-state 與 instances 同時給了"
                f"(instances={cfg.instances!r},來源 {cfg.sources.get('instances')})。"
                f"只留一個。")
        triples, state_env = instances_from_state(args.from_state)
    else:
        triples, state_env = parse_instances(cfg.instances, str(cfg.profile)), {}
    links = [Link(inst, prof, cfg, port) for inst, prof, port in triples]
    validate_fleet_ports(links)
    ser_off = {0: 0, 1: 2, 2: 3, 5: 5, 6: 6, 7: 7, 8: 8}.get(int(cfg.serial), "?")
    print(f"\n埠位(SERIAL{cfg.serial} 偏移 +{ser_off};算術出處 SITL_cmdline.cpp:251/434-437 "
          f"+ SITL_State.h:29-38。⚠️ 啟動器給過 --base-port 的話算術**不成立**,見檔頭 §2.5):")
    for l in links:
        print(f"  {l.key:<6} profile={l.profile:<7} {l.conn_str:<26} [{l.port_src}]")

    # ── 權威對照表:--ports-json 明給,或由 state 檔的 VTOL_WORLD 推出來 ──
    # ⚠️ 推出來的路徑**不存在時要說**,不要安靜地當作沒給 —— 「拿不到就照舊」正是
    #    本檔在防的那個形狀。
    ports_json = args.ports_json
    if not ports_json and state_env.get("VTOL_WORLD"):
        cand = state_env["VTOL_WORLD"] + ".ports.json"
        if os.path.exists(cand):
            ports_json = cand
            print(f"\n[ports] 由 state 檔的 VTOL_WORLD 推出對照表:{cand}")
        else:
            print(f"\n⚠️ state 檔的 VTOL_WORLD={state_env['VTOL_WORLD']} 推不到對照表"
                  f"({cand} 不存在)⇒ PWM 那兩條契約會判『未驗證』。"
                  f"用 --ports-json 明講,或先確認世界檔是不是被搬走了。")
    if ports_json:
        apply_ports_json(links, ports_json, cfg)
    else:
        print("\n⚠️⚠️ 沒有 ports.json:Q_M_PWM_MIN/MAX(vtol)與 MOT_PWM_MIN/MAX(copter)"
              "會判『未驗證 ⇒ 違規』。這兩條的期望值是**那一架 SDF 自己的** "
              "servo_min/servo_max,不是常數 —— 猜一個常數會在真的對不上時亮綠燈"
              "(上游 iris 是 1100/1900、我們的 VTOL 是 1000/2000,"
              "而後者還能被 GZ_VTOL_LIFT_PWM_MIN/MAX 改掉)。")

    fleet = Fleet(links, cfg)
    rep = Report()
    try:
        ok = step_connect(fleet, rep, cfg)
        if ok or fleet.live:
            step_streams(fleet, rep, cfg)
            step_params(fleet, rep, cfg, expect_table)
            step_origin(fleet, rep, cfg)

        if args.mode == "watch":
            cmd_watch(fleet, cfg)
        elif args.mode == "mission":
            if step_arm(fleet, rep, cfg, args.dry_run):
                if step_takeoff(fleet, rep, cfg, args.dry_run, args.takeoff_command_int):
                    step_land(fleet, rep, cfg, args.dry_run)
                else:
                    # ⚠️ 起飛沒到位也要降落 —— 留一架解鎖著的飛機在 world 裡,
                    #    下一個人接手時看到的是「怎麼會有東西在動」。
                    print("\n⚠️ 起飛未達標,仍執行降落收尾。")
                    step_land(fleet, rep, cfg, args.dry_run)
            elif not args.dry_run and any(l.armed for l in fleet.live):
                # ⚠️⚠️ 原版這條路徑**沒有收尾**:step_arm 只要回 False 就直接跳過
                #    起飛與降落 —— 但「回 False」包含「16 架裡 3 架切模式失敗」這種
                #    部分成功,另外 13 架已經解鎖了。留著解鎖的飛機在 world 裡,
                #    與上面那個註解在防的是同一件事,只是漏了一條分支。
                #    (「有一半成功」在這條線上比全失敗更常見,見檔頭 §2.5。)
                armed_now = [l for l in fleet.live if l.armed]
                print(f"\n⚠️⚠️ 解鎖步驟未全數通過,但已有 {len(armed_now)} 架是解鎖狀態"
                      f"({', '.join(l.key for l in armed_now)})⇒ 仍執行收尾,"
                      f"不留解鎖中的飛機在 world 裡。")
                step_land(fleet, rep, cfg, args.dry_run)
        elif args.mode == "goto":
            missing = [n for n, v in (("--lat", args.lat), ("--lon", args.lon),
                                      ("--alt", args.alt),
                                      ("--loiter-radius", args.loiter_radius)) if v is None]
            if missing:
                raise SystemExit(f"✗ goto 需要 {', '.join(missing)}")
            cmd_goto(fleet, rep, cfg, args.lat, args.lon, args.alt, args.loiter_radius,
                     args.reposition_change_mode, args.yaw)
    except KeyboardInterrupt:
        print("\n⚠️ 使用者中止。")
        rep.failed = True
    finally:
        fleet.close()

    print("\n" + "═" * 72)
    dead = [l for l in fleet.links if l.dead]
    if dead:
        print("鏈路死亡:")
        for l in dead:
            print(f"  ✗ {l.key} {l.conn_str} — {l.dead_reason}")
    print(f"總結:{'全部通過' if not rep.failed else '**有項目失敗**(逐機明細在上面)'}")
    print("═" * 72)
    # ⚠️ 只有明確要求才輸出 JSON。原本寫成「stdout 不是 tty 就自動吐」——
    #    那會讓每一次 `| tee`、`| tail` 都在人看的報表後面再貼一份幾十 KB 的 JSON,
    #    把真正的紅字沖走。要機器讀的格式就明講:`--report <檔案>` 或 `--report -`。
    if args.report:
        rep.dump(None if args.report == "-" else args.report, cfg,
                 {"instances": [{"instance": l.instance, "profile": l.profile,
                                 "conn": l.conn_str, "sysid": l.sysid,
                                 "dead": l.dead, "reason": l.dead_reason}
                                for l in fleet.links]})
    if rep.failed and bool(cfg.strict):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
