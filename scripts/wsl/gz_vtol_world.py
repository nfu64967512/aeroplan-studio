#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_vtol_world.py — 產生「同一個 world、多架、可混機種」的 Gazebo Harmonic 世界檔。

取代 `gz_gen_world.py` 的做法。⚠️ 兩者的差別是**結構上的**,不是參數調整:

  舊做法(gz_gen_world.py:64 `make_model()`):每架**複製整個模型目錄**再改
    `fdm_port_in`,世界檔用 N 個 `<include>` 掛進來。
  新做法(本檔):**只 include 最底層的裸模型**(例如 `model://iris_with_standoffs`),
    把每一架的 model 內文(含它自己那一份 `ArduPilotPlugin`)**內嵌展開**進同一個
    world 檔,每個外層 `<model name="...">` 各自帶一份 plugin。

為什麼換:2026-08-11 記下的「兩份複本 include 同一底層模型會撞名」**已被推翻**。
本機實證(~/swarm/gazebo/swarm3.sdf,唯讀參考):四個外層 model(leader1 /
follower1..3)各帶一份 ArduPilotPlugin、fdm_port_in 9002/9012/9022/9032、四個都
lock_step 1,而且**四個都 include 同一個** `model://iris_with_standoffs`;實測
/stats 的 sim_time 前進、RTF 0.236。所以撞名不是問題,複製模型目錄那條路是多餘的
——而且它有害:每複製一份就多一份會走樣的副本,埠改錯了也看不出來。

────────────────────────────────────────────────────────────────────────
⚠️⚠️⚠️ 三個會讓你「看起來成功、其實整條鏈是死的」的坑,全部寫在這裡
────────────────────────────────────────────────────────────────────────

【坑 1】埠與 SITL instance 是**綁死**的,不能各算各的。
  ArduPilot 的 JSON 後端從 instance 號自己推埠:
      libraries/AP_HAL_SITL/SITL_cmdline.cpp:258-259   SIM_IN_PORT=9003 / SIM_OUT_PORT=9002
      libraries/AP_HAL_SITL/SITL_cmdline.cpp:444-449   若仍等於預設值就 += instance*10
  ⇒ `-I0` 送到 9002、`-I5` 送到 9052、`-I10` 送到 9102。
  SDF 的 `<fdm_port_in>` **必須等於**那架 SITL 由 instance 推出來的埠。
  ⇒ 所以本產生器的介面收的是**每架的 SITL instance 號**,fdm 埠由
     `FDM_PORT_BASE + FDM_PORT_STRIDE * instance` 算出來,**不是**由 uav 序號算。
  舊的 gz_gen_world.py:107 用 1-based 的 uav 序號算(`9002 + 10*(i-1)`),
  只有在 SITL 剛好用 -I0,-I1,... 時才碰巧對得上。錯位的症狀是 SITL 一直印
  "No JSON sensor message" —— 跟「模型撞名」的症狀**一模一樣**,極容易誤診。

【坑 2】⚠️⚠️ `ss -lunp` 看到「埠都 bind 了」**證明不了任何事**。
  ArduPilotPlugin 的 socket 是 `SocketUDP sock = SocketUDP(true, true)`
  (~/ardupilot_gazebo/src/ArduPilotPlugin.cc:227),第二個 true 走
  `SocketUDP::set_reuseaddress()`(~/ardupilot_gazebo/src/SocketUDP.cc:72-75)設
  SO_REUSEADDR。本機實測:兩個 SO_REUSEADDR socket 都能 bind 127.0.0.1:19003 成功,
  **後 bind 的收到全部封包、先 bind 的收到零**,無錯誤、無 log。
  ⇒ 重複的 fdm 埠在我們現有的每一層檢查裡都是**隱形**的。
  ⇒ 本產生器因此把「fdm 埠必須逐架唯一」做成**硬失敗**(見 `plan_fleet()`),
     這是唯一能在啟動前擋下它的地方。真正的執行期驗證要靠啟動器**實際送探測封包
     並確認誰收到**,不能只看 ss。

【坑 3】⚠️⚠️ 判準要換:「sim_time 有前進」是必要條件但**不充分**。
  lock_step 的失效語意(ArduPilotPlugin.cc:1204-1213 的 while 迴圈,
  以及 arduPilotOnline 的設定處):
    * 已經上線過的機體斷線 ⇒ arduPilotOnline 永遠不會被設回 false ⇒ PreUpdate 卡在
      `while (!ReceiveServoPacket() && arduPilotOnline)`,每輪睡 10 ms。gz-sim 的
      PreUpdate 跑在**單一更新執行緒** ⇒ **一架死掉會凍結整個 world**。而且因為是
      sleep 不是 busy-spin,**沒有 CPU 尖峰**,用 CPU 判別會失效。
    * 從來沒收到第一個封包的機體 ⇒ arduPilotOnline 仍是 false ⇒ 迴圈立刻退出 ⇒
      **world 照跑,那架靜靜死掉**。
  ⇒ 要證明每一架都活著,必須**逐機**檢查(啟動器的責任,不是本檔的):
     每架 SITL 的 log 有 "JSON received" 且**沒有**持續的 "No JSON sensor message",
     或逐機讀 MAVLink 的姿態/位置在變。
  本檔能做的是把「第 k 架 = instance I = fdm 埠 P」寫成一份機器可讀的對照表
  (`<world>.ports.json`),讓啟動器與 coordinator 用**同一份**,不要各算各的。

────────────────────────────────────────────────────────────────────────
2026-09-09 對抗式覆審改掉的東西(留紀錄,免得有人「修回去」)
────────────────────────────────────────────────────────────────────────

【改 1】拿掉兩個**假守衛**——它們擋掉的是合法寫法,擋不到任何真 bug,而且
  對本 repo 真正要飛的 sitl/gazebo/models/quadplane_vtol/model.sdf **整個產不出檔**:
    (a)「同一個 <control channel="N"> 出現兩次就硬失敗」。錯:ArduPilotPlugin 的
        controls 是 vector(ArduPilotPlugin.cc:82-99 / :539-573 / :801),
        :1648-1652 逐個取 _pwm[controls[i].channel] ⇒ **多個 control 共用一個
        PWM channel 完全合法**,兩塊副翼 / 兩片 elevon / V 尾就是這樣接的。
        我們的 VTOL 正是 channel 0 掛 aileron_left_joint + aileron_right_joint。
        改成:允許共用,但共用者的 servo_min/servo_max 必須一致(不一致才是真 bug)。
    (b)「模型自帶頂層 <pose> 就硬失敗、叫作者刪掉」。錯:model.sdf 要能被單獨
        <include>,自帶 <pose> 是正常的;而且我們的模型那個 pose 帶著**唯一有出處**
        的起落架高度(0.25,註解寫明最低點在 z=−0.25)。改成:丟掉它、大聲說丟了什麼,
        並在沒有更明確來源時**以它為準**(側寫的 0.30 是無出處佔位值)。

【改 2】補上真正會咬的守衛:
    (a)【坑 7】的 check_lift_pwm 原本用 `{c["channel"]: c}` 建索引 ⇒ 一個 channel
        掛多個 control 時只留最後一個,前面的**靜靜沒被檢查**。改成逐 control 全查,
        並把「實際檢查了幾個」印出來 + 寫進 ports.json。
    (b) plan_fleet 原本只保證機隊**內部**埠不重複,擋不到「跟這台機器上別人的航次撞」。
        2026-09-09 `ss -lunp` 實測:UDP 9002/9012/9022/9032(gz,pid 161034)、
        9005/9015/9025/9035、5501/5511/5521/5531 全被另一個 session 佔著。
        ⇒ 加 instance 下限(MIN_INSTANCE_DEFAULT)+ 唯讀的 `ss -lun` 佔用檢查。
        ⚠️ 這道檢查是**單向**的,認識論寫在 probe_busy_udp_ports() 的 docstring 裡。
        ⚠️ 我自己第一版把欄位寫成 cols[4](那是 Peer,恆為 "0.0.0.0:*")⇒ 永遠解析出
           零個埠 ⇒ **恆綠**。已修成 cols[3],並加上「解析不出任何埠就報成沒跑成」。
    (c) 寫檔順序原本是 gz 驗 → os.replace → 讀回 → 寫 ports.json ⇒ 讀回一丟例外就會
        留下「新世界檔 + 舊 ports.json」,而那正是【坑 1】的形狀且完全不報錯。
        改成全部驗完才一次換上去,舊 ports.json 先移走。
    (d) 模型檔的 sha1:同一個檔在一次執行中被解析 3~5 次,而另一個 session 正在寫它
        (實測:我第一次 ls 只有 model.config,兩分鐘後 model.sdf 出現且當下是壞 XML)。
        產完再雜湊一次比對,不一致就整份作廢。

【改 3】對照表補齊 + 讀回表不再說謊:
    * 漏掉的埠:_rcin_port 5501+10i 與 _fg_view_port 5503+10i
      (SITL_cmdline.cpp:254-256 / :435-442),ss 實測它們真的被佔著。
    * with_runway 原本恆定回報「來源 = cli」;改成走 Settings 並補 GZ_VTOL_RUNWAY。
    * sysid 的**參數名**:這棵樹是 MAV_SYSID,SYSID_THISMAV 已不在參數表
      (ArduPlane/Parameters.cpp:17 與 :1578 的轉換表),寫錯會被韌體靜默略過。

────────────────────────────────────────────────────────────────────────
其他已經踩過的坑(與本檔相關的部分)
────────────────────────────────────────────────────────────────────────

⚠️ `gz sdf -k` 驗有 `<include>` 的世界檔時,關鍵環境變數是 **SDF_PATH**,
   **不是** GZ_SIM_RESOURCE_PATH。本機 2026-09-09 實測(gz / libsdformat14.so.14.9.0):
       (無環境變數)                    → Error Code 14: Unable to find uri[model://...]  exit=255
       GZ_SIM_RESOURCE_PATH=<models>   → 同上,一樣失敗                                 exit=255
       SDF_PATH=<models>               → "Valid."                                       exit=0
   ⇒ `gz_up.sh` 檔頭那段「不要用 gz sdf -k,它無法解析 model://」的註解**是錯的**,
      它只是漏了 SDF_PATH。本檔不改 gz_up.sh(不是我的檔),但在這裡留下反證。
   ⇒ 所以本產生器**自己會跑** `gz sdf -k`,驗不過就硬失敗,絕不產出一個壞檔。

⚠️ `model://ground_plane` 本機**沒有**,gz 預設會去 Fuel 線上抓,離線環境直接
   Error Code 14 讓整個世界起不來(實測 2026-08-11)。⇒ 地面一定要**內嵌**。

⚠️⚠️ 桃園世界檔有個**未解**的基準問題:地面在世界 z=0,而 spherical_coordinates 的
   elevation 寫 32 m ⇒ **Gazebo 裡的「海平面」其實是 32 m AMSL**。而打擊線的紅心是
   釘死 0 m AMSL(記憶 sea-level-target-datum)。這次先照 32 做,但產生的世界檔裡
   會留一行註解標出來,免得下一個人重演「殘餘高度恆定」那一類的地形落差 bug。

⚠️ GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH 只寫在 ~/.bashrc ⇒ **非互動
   shell 讀不到**,而 gz 的錯誤訊息不會指向這個原因。啟動器要明確 export。
   本檔不依賴它們(驗證只用 SDF_PATH,而且是自己組出來明確傳進子行程的)。

⚠️ 這台機器上同時有別人的航次在跑(另一個 session 的四機 Gazebo)。
   起 gz 時**一定要設 GZ_PARTITION**(建議 vtolswarm)與它隔離;gz_up.sh 完全沒有
   這個概念。本檔會把建議的 partition 寫進 ports.json 與啟動提示。
   ⚠️ 本檔**只寫檔,不啟動任何東西**;唯讀的 `gz sdf -k` 除外。

⚠️ 產生檔必須是**確定性**的:同樣輸入跑兩次 md5 要一樣。本專案踩過
   `str(callable)` 含記憶體位址造成假 diff 的坑(記憶 export-must-be-deterministic)。
   ⇒ 本檔:不寫時間戳、不寫路徑以外的環境資訊、浮點一律定寬格式化(`_f()`)、
      並在產生後**再建一次**比對文字是否逐字相同(`--no-determinism-check` 可關,
      但預設開著)。

⚠️ CRLF:本檔在 Windows 端的 repo 裡,但**在 WSL 執行**。所有輸出一律
   `newline="\n"`,不要讓 Windows 的 Python 寫出 CRLF(記憶 crlf-breaks-wsl-scripts)。

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

  # 2 台 VTOL + 2 台 iris,instance 從 10 起(10/11/12/13)
  python3 gz_vtol_world.py --add quadplane_vtol:2 --add iris:2 \
      --instance-base 10 --out ~/vtolswarm/worlds/taoyuan_vtol.sdf

  # 逐架指定(混機種、各自的 instance 與位置)
  python3 gz_vtol_world.py \
      --vehicle profile=quadplane_vtol,instance=10,name=vtol1,x=0,y=0 \
      --vehicle profile=iris,instance=12,name=iris1,x=-40,y=0 \
      --out ~/vtolswarm/worlds/mixed.sdf

  # 只看對照表,不寫檔
  python3 gz_vtol_world.py --add iris:4 --instance-base 10 --dry-run

產物:
  <out>              世界檔(gz sdf -k 已驗過)
  <out>.ports.json   對照表:第 k 架 = instance I = fdm 埠 P,以及逐通道的 PWM 刻度
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# ══════════════════════════════════════════════════════════════════════
# 常數。每一個寫死的數字都要有出處。
# ══════════════════════════════════════════════════════════════════════

# 【出處】AP_HAL_SITL/SITL_cmdline.cpp:258-259 與 :444-449
#   SIM_OUT_PORT=9002(SITL 送 FDM 出去 / Gazebo 收)
#   SIM_IN_PORT =9003(SITL 收 Gazebo 回來的狀態)
#   IRLOCK_PORT =9005
#   三者都在 instance != 0 時 += instance*10。
FDM_PORT_BASE = 9002        # = SIM_OUT_PORT,對應 SDF 的 <fdm_port_in>
SITL_SIM_IN_BASE = 9003     # = SIM_IN_PORT
IRLOCK_PORT_BASE = 9005     # = IRLOCK_PORT
PORT_STRIDE = 10            # instance*10

# 【出處】SITL_cmdline.cpp:254-256 宣告、:435-442 做 += _instance*10。
#   ⚠️ 原本這張表只記了 fdm / sim_in / irlock / TCP 三口,**漏了這兩個**:
#      _rcin_port  = 5501 + 10*instance   (UDP,RC 輸入)
#      _fg_view_port = 5503 + 10*instance (UDP,FlightGear 視角輸出)
#   2026-09-09 本機 `ss -lun` 實證它們是真的被佔著的:另一個 session 的四台
#   arducopter 佔了 5501/5511/5521/5531。一張自稱「單一真相來源」的對照表漏掉
#   兩個會被佔用的埠,下一個人就會拿它去排埠然後撞上——所以補進來。
SITL_RCIN_PORT_BASE = 5501
SITL_FGVIEW_PORT_BASE = 5503

# 【出處】ArduPilotPlugin.cc:1280-1283 —— <fdm_port_out> 已 deprecated;
#   plugin 只 bind <fdm_port_in>(:1286),回傳狀態是 sendto 回**封包來源**(:2031)。
#   ⇒ 世界檔裡只需要也只能設 fdm_port_in,不要再寫 fdm_port_out。
FDM_ADDR_DEFAULT = "127.0.0.1"

# 【出處】記憶/實測的埠位表:SITL master TCP 5760+10i、直連 5762+10i、
#   第三路 5763+10i(每個 instance 佔三個 TCP)。這裡只用來**報告**,
#   ⚠️ 不是本檔能決定的東西(見 ports.json 的 warnings)。
SITL_TCP_SERIAL0_BASE = 5760
SITL_TCP_SERIAL1_BASE = 5762
SITL_TCP_SERIAL2_BASE = 5763

# 【出處】taoyuan_runway.sdf 檔頭的 SWARM_LOC="25.0777,121.2325,32,52"
#   (swarm_sitl_up.sh 產生的 swarm_state.env)。SITL 的 --home 必須與此一致,
#   否則 Gazebo 的地理座標與飛控 HOME 對不上,所有 lat/lon 相關驗收失去意義。
ORIGIN_LAT = 25.0777
ORIGIN_LON = 121.2325
ORIGIN_ELEV_M = 32.0
ORIGIN_HDG_DEG = 52.0       # 跑道/隊列的羅盤航向

# 【出處】taoyuan_runway.sdf:「<spherical_coordinates> 的 heading_deg 保持 0。
#   它旋轉的是世界座標框,不是飛機航向。」上游模板 zephyr_runway.sdf 也是 0。
SPHERICAL_HEADING_DEG = 0.0

# 【出處】swarm3.sdf 的 <physics> 註解(唯讀參考,別人量到的):
#   步長必須 1 ms。改 2.5 ms 可把 RTF 從 0.15 拉到 0.38,但陀螺取樣率掉到讓
#   ArduCopter 速率環進滾轉極限環(max|R| 386 deg/s vs 指令 89、馬達混控 12% 取樣
#   rail-to-rail 飽和),滿油門仍一路沉到地面。回 1 ms 後 max|R| 55、飽和 0%。
#   這是正確性換速度,不要為了跑快改回去。
PHYSICS_STEP_SIZE = 0.001
PHYSICS_RTF = 1.0

# 【出處】taoyuan_runway.sdf:「本任務要飛 60 km,地面不能只有跑道那一塊」⇒ 200 km。
GROUND_SIZE_M = 200000.0

# 【出處】taoyuan_runway.sdf 的跑道 pose 推導:
#   ENU 的 yaw 從 +X(東)逆時針量,羅盤航向 H 對應 yaw = 90 − H。
#   上游模板 zephyr_runway.sdf 的跑道是 pose (−29, 545) 配 yaw 363。
#   **旋轉跑道時位移必須一起轉**,否則跑道會從機體底下滑開(實測 2026-08-11:
#   忘了轉位移 ⇒ 原點下面變空的 ⇒ SIMSTATE 抓到 roll=3.141 / pitch=1.544)。
RUNWAY_TEMPLATE_X = -29.0
RUNWAY_TEMPLATE_Y = 545.0
RUNWAY_TEMPLATE_YAW = 363.0

# 隊列預設間距。⚠️ 這個數字**沒有上游出處**,是自訂值:VTOL 垂直起降有旋翼下洗流,
#   舊 gz_gen_world.py 的 30 m 是給固定翼縱列用的,swarm3.sdf 的多旋翼用 16 m。
#   取 20 m 折衷,可用 --spacing / GZ_VTOL_SPACING 覆寫。
DEFAULT_SPACING_M = 20.0

# 【出處】ArduPilotPlugin 的 control channel 上限實際是 16(0-15)。
MAX_CONTROL_CHANNEL = 15

# 【出處】gz sim 的 transport partition。⚠️ 這台機器上同時有別人的四機 Gazebo 在跑,
#   不隔離會互相看到對方的 topic/service。名稱自訂。
DEFAULT_GZ_PARTITION = "vtolswarm"

# ⚠️⚠️⚠️ instance 下限。2026-09-09 `ss -lunp` 實測,另一個 session 的四機 Gazebo
#   正佔著 UDP 9002/9012/9022/9032(gz server,pid 161034)與 9005/9015/9025/9035
#   (arducopter),TCP 5760…5793,UDP 5501/5511/5521/5531。
#   ⇒ instance 0..3 產出的埠**現在就是撞的**,而依【坑 2】那是完全靜默的:
#      SO_REUSEADDR 讓兩邊都 bind 成功,後 bind 的收全部、先 bind 的收零。
#   ⇒ 所以本產生器對 instance < MIN_INSTANCE_DEFAULT 硬失敗。要用低號碼必須
#      明講(--min-instance 0),讓「我知道我在做什麼」變成一個看得見的決定。
MIN_INSTANCE_DEFAULT = 10


# ══════════════════════════════════════════════════════════════════════
# 機種側寫(profile)
#
# ⚠️ 刻意**不把模型內文寫死在 Python 裡**。profile 只說「去哪個模型目錄拿 model.sdf」,
#    內文原封不動搬進世界檔。理由:
#      (a) iris 那一份內文有 ~880 行(4 個 lift-drag + 4 個 apply-joint-force +
#          ArduPilotPlugin 的 4 個 control),抄進 Python 就等於多一份會走樣的副本;
#      (b) VTOL 的模型由另一個 session 產出,本檔不該對它的內容做任何假設,
#          只做**讀回式驗證**(見 inspect_model_body())。
# ══════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class Profile:
    """一個機種側寫。model_dir 是模型目錄名,會在 --model-path 上逐段尋找。"""
    key: str
    model_dir: str
    name_prefix: str
    # 生成高度。⚠️ 出處見各 profile 的註解;不對會直接穿地或懸空掉下來。
    spawn_z: float
    # 模型自己的座標慣例。⚠️ iris 與 zephyr **不同**,一台機體只能有一個。
    spawn_roll_deg: float
    spawn_pitch_deg: float
    # 升力馬達所在的 control channel。空 tuple = 這個機種沒有「升力馬達」這個概念
    # (例如純多旋翼,四顆都是主升力,PWM 刻度由上游 iris 模型自己決定)。
    lift_channels: Tuple[int, ...]
    note: str


# ⚠️⚠️ 【坑 7:PWM 刻度】iris 的 SDF 四個 control 都是 servo_min 1100 / servo_max 1900,
#   而 ArduPilot 的 quadplane.parm 與我們的 generate_vtol_params() 都是
#   Q_M_PWM_MIN/MAX = 1000/2000。照抄 iris 的 control 區塊 ⇒
#   ArduPilotPlugin.cc:1676-1678 的 `raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min)`
#   接 `clamp(raw_cmd, 0, 1)` 會把底部 12.5% 全夾成 0、頂部 12.5% 全夾成 1,
#   四顆一起損失 25% 行程。
#   ⇒ VTOL 的升力馬達 servo_min/servo_max 必須寫 1000/2000。
#   ⚠️ 另注意 Q_M_PWM_TYPE 預設 NORMAL ⇒ SERVOn_MIN/MAX 對馬達輸出**不生效**,
#      生效的是 Q_M_PWM_MIN/MAX。
LIFT_PWM_MIN_DEFAULT = 1000
LIFT_PWM_MAX_DEFAULT = 2000

# 【出處:通道映射,已逐檔驗證】
#   端到端鏈是 SERVO(N+1) → servos[N] → pkt.pwm[N] → SDF 的 control channel="N"。
#   ArduPlane 預設 SERVO1-4 = aileron / elevator / throttle / rudder
#     ⇒ channel 0=副翼、1=升降舵、2=前推馬達、3=方向舵
#   QuadPlane 升力馬達 = SERVO5-8(SIM_QuadPlane 的 motor_offset 預設 4)
#     ⇒ channel 4/5/6/7
#   `--model JSON` 時 SIM_QuadPlane 不會被實例化,真正生效的是 SERVOn_FUNCTION;
#   ArduPlane 預設把 motor1..4 指到 CH_5..CH_8(quadplane.cpp:667-671 的 runtime 預設)。
QUADPLANE_LIFT_CHANNELS = (4, 5, 6, 7)

PROFILES: Dict[str, Profile] = {
    # 我們自己的 VTOL。⚠️ 這個模型目錄由**另一個 session** 產出,本檔寫這行的時候
    #   它還不存在(2026-09-09 實測:~/ardupilot_gazebo/models 底下 11 個目錄,
    #   帶 ArduPilotPlugin 的只有 iris / iris_with_gimbal / zephyr /
    #   zephyr_with_parachute,**沒有任何 QuadPlane / VTOL 模型**)。
    #   2026-09-09 對照過對方已落地的 model.config:目錄名是 `quadplane_vtol`,
    #   位置 <repo>/sitl/gazebo/models/,而且是**自足式**的(不 include 任何其他模型、
    #   不參照 mesh,離線可載入)⇒ 本產生器內嵌它的內文時不會再產生第二層 model:// 相依。
    #   ⚠️ 這裡刻意讓 model_dir 與 profile key 同名:名字對不上時 resolve_profile()
    #      的 fallback 會用同一個字串,兩條路殊途同歸,少一個對不上的失敗模式。
    "quadplane_vtol": Profile(
        key="quadplane_vtol",
        model_dir="quadplane_vtol",
        name_prefix="vtol",
        # ⚠️ 這是**佔位值,無出處**(寫的時候模型還沒生出來)。2026-09-09 模型落地後
        #    實測它自己的頂層 <pose> 是 `0 0 0.25 0 0 0`,而且註解寫明「起落架最低點
        #    在 base_link 座標 z=−0.25」⇒ 權威值是 0.25 不是 0.30。
        #    ⇒ 現在的規則是:模型檔自己的 <pose> 優先(見 Vehicle.z_source),
        #      這個欄位只在模型沒寫 <pose> 時才會被用到。**不要**把 0.25 抄到這裡
        #      再多一份會走樣的副本 —— 同一個常數活在兩個地方就是本專案的老毛病。
        spawn_z=0.30,
        # ⚠️ iris 用 modelXYZToAirplaneXForwardZDown = "0 0 0 180 0 0",
        #    zephyr 用 "0 0 0 180 0 -90" —— 兩者**不同**,一台機體只能有一個。
        #    zephyr 因此在世界檔裡要 roll −90(見 taoyuan_runway.sdf 的佔位機體)。
        #    混用零件做出來的 VTOL 選了哪一個,要由模型作者在模型檔註解裡寫明。
        #    ✔ 2026-09-09 已對模型檔讀回確認:它寫的是
        #      `<modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</...>`
        #      (iris 慣例,x=前),註解也說明「我們沒有 mesh 包袱,直接用航空慣例」
        #      ⇒ roll 0 是對的,不是猜的。
        spawn_roll_deg=0.0,
        spawn_pitch_deg=0.0,
        lift_channels=QUADPLANE_LIFT_CHANNELS,
        note="ArduPlane + Q_ENABLE=1;channel 0-3 = 副翼/升降舵/前推/方向舵,4-7 = 升力馬達",
    ),
    # 上游 iris(ArduCopter)。用來做「同一個 world 混機種」的對照組。
    "iris": Profile(
        key="iris",
        model_dir="iris_with_ardupilot",
        name_prefix="iris",
        # 【出處】~/swarm/gazebo/swarm3.sdf 的 leader1 pose z = 0.195(實測會動的值)。
        spawn_z=0.195,
        spawn_roll_deg=0.0,
        spawn_pitch_deg=0.0,
        # 空 tuple:iris 的四顆是主升力,PWM 刻度沿用上游 1100/1900。
        # ⚠️ 上游 iris 其實也有【坑 7】那個 25% 行程損失(ArduCopter 的
        #    MOT_PWM_MIN/MAX 預設 1000/2000),但那是上游的模型、也不是我們的量測線,
        #    本檔**不擅自改別人的模型**,只在 ports.json 把實際刻度讀回來報告。
        lift_channels=(),
        note="上游 ArduCopter iris,混機種對照組;PWM 刻度沿用上游 1100/1900",
    ),
    # 上游 zephyr(ArduPlane 飛翼)。留著是因為它是本機唯一另一個帶 plugin 的固定翼,
    # 拿來驗「異質機隊」時 plane 與 copter 的串流預設差異最方便。
    "zephyr": Profile(
        key="zephyr",
        model_dir="zephyr_with_ardupilot",
        name_prefix="zephyr",
        # 【出處】taoyuan_runway.sdf 的佔位機體 pose "0 0 0.422 -90 0 -145"。
        spawn_z=0.422,
        # ⚠️ roll −90 是 zephyr 模型自己的座標慣例,**不要動**;
        #    要改航向只動 yaw(SDF 的 pose 是 R = Rz(yaw)·Ry(pitch)·Rx(roll),
        #    yaw 最後才繞世界 Z 軸轉,所以單改 yaw 就是純改航向)。
        spawn_roll_deg=-90.0,
        spawn_pitch_deg=0.0,
        lift_channels=(),
        note="上游 ArduPlane zephyr 飛翼;⚠️ 座標慣例與 iris 不同(roll −90)",
    ),
}


def resolve_profile(key: str) -> Profile:
    """把 profile 名字換成 Profile。認不得的名字就當成**模型目錄名**直接用。

    ⚠️ 這條 fallback 是刻意的:VTOL 模型由另一個 session 產出,目錄名可能與
       PROFILES 裡預寫的不同。與其硬失敗逼人改本檔,不如讓它能直接用目錄名,
       並在報告裡標明「這個 profile 沒有內建側寫,幾何參數全走預設」。
    """
    if key in PROFILES:
        return PROFILES[key]
    return Profile(
        key=key,
        model_dir=key,
        name_prefix="".join(ch for ch in key if ch.isalnum()) or "uav",
        spawn_z=0.30,
        spawn_roll_deg=0.0,
        spawn_pitch_deg=0.0,
        lift_channels=(),
        note="⚠️ 未內建側寫,直接當模型目錄名使用;spawn_z/roll 走預設,請用 CLI 覆寫確認",
    )


# ══════════════════════════════════════════════════════════════════════
# 設定解析 + 讀回
#
# 本專案硬性要求:「設定值必須讀回驗證,不接受靜默失敗」。
# 這個小類別把「每個可調常數的**生效值**與它**從哪來**」記下來,一律印出來,
# 並寫進 ports.json。不做這件事的話,一個打錯的環境變數會安靜地走預設值。
# ══════════════════════════════════════════════════════════════════════

def _as_bool(v) -> bool:
    """給 Settings.get 用的布林 cast。

    ⚠️ 不可以用 `bool(os.environ[...])`:`GZ_VTOL_RUNWAY=0` 會變成 True(非空字串),
       這種「設了等於沒設」是本專案最常見的靜默失敗形狀之一。認不得的字面值一律
       硬失敗,不猜。
    """
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    raise ValueError("要 1/0/true/false/yes/no/on/off,收到 %r" % v)


@dataclass
class SettingRow:
    key: str
    value: object
    source: str      # "cli" / "env:GZ_VTOL_XXX" / "default"
    env_name: str


class Settings:
    def __init__(self) -> None:
        self.rows: List[SettingRow] = []

    def get(self, key: str, cli_value, env_name: str, default, cast):
        """cli > 環境變數 > 預設。三層都記下 provenance。"""
        if cli_value is not None:
            value, source = cast(cli_value), "cli"
        else:
            raw = os.environ.get(env_name)
            if raw is not None and raw != "":
                try:
                    value = cast(raw)
                except Exception as exc:      # noqa: BLE001
                    # ⚠️ 環境變數打錯絕不可以退回預設值 —— 那正是「靜默吃到舊參數」。
                    raise SystemExit(
                        "✗ 環境變數 %s=%r 無法解析成 %s:%s"
                        % (env_name, raw, getattr(cast, "__name__", "值"), exc))
                source = "env:%s" % env_name
            else:
                value, source = cast(default), "default"
        self.rows.append(SettingRow(key=key, value=value, source=source,
                                    env_name=env_name))
        return value

    def dump(self, stream=sys.stdout) -> None:
        print("[cfg] 生效設定(讀回):", file=stream)
        width = max((len(r.key) for r in self.rows), default=1)
        for r in self.rows:
            print("[cfg]   %-*s = %-28s  ← %s (env %s)"
                  % (width, r.key, r.value, r.source, r.env_name), file=stream)

    def as_json(self) -> List[dict]:
        return [{"key": r.key, "value": r.value, "source": r.source,
                 "env": r.env_name} for r in self.rows]


# ══════════════════════════════════════════════════════════════════════
# 埠算術 —— 【坑 1】的唯一真相來源
# ══════════════════════════════════════════════════════════════════════

def ports_for_instance(instance: int) -> Dict[str, int]:
    """由 SITL instance 號推出這一架佔用的所有埠。

    ⚠️ **不要**用 uav 序號算。出處與理由見檔頭【坑 1】。
    """
    if instance < 0:
        raise SystemExit("✗ instance 不能是負數:%d" % instance)
    off = PORT_STRIDE * instance
    return {
        # 硬事實:由韌體的 instance 算術決定,SDF 必須跟著它。
        "fdm_port_in": FDM_PORT_BASE + off,          # Gazebo bind,SITL 送過來
        "sitl_sim_in": SITL_SIM_IN_BASE + off,       # SITL bind,Gazebo 送回去
        "irlock_port": IRLOCK_PORT_BASE + off,       # SITL 佔著,即使沒用到
        # ⚠️ 這兩個原本漏掉了(SITL_cmdline.cpp:254-256 / :435-442),而 ss 實測
        #    另一個 session 的 5501/5511/5521/5531 是真的被佔著的。
        "sitl_rcin_port": SITL_RCIN_PORT_BASE + off,
        "sitl_fgview_port": SITL_FGVIEW_PORT_BASE + off,
        # 韌體預設的 TCP 三口(若啟動器不覆寫 serial0 就是這些)。
        "sitl_tcp_serial0_default": SITL_TCP_SERIAL0_BASE + off,
        "sitl_tcp_serial1_default": SITL_TCP_SERIAL1_BASE + off,
        "sitl_tcp_serial2_default": SITL_TCP_SERIAL2_BASE + off,
    }


def udp_ports_for_instance(instance: int) -> Dict[str, int]:
    """這一架會被**某個行程 bind** 的 UDP 埠(拿去跟 `ss -lun` 比對用)。

    ⚠️ sitl_sim_in / irlock / rcin / fgview 是 SITL 那一側 bind 的,fdm_port_in 是
       Gazebo 那一側 bind 的。四個都要比,因為撞到任何一個都會出事,而且
       **全部都是靜默的**。
    """
    p = ports_for_instance(instance)
    return {k: p[k] for k in ("fdm_port_in", "sitl_sim_in", "irlock_port",
                              "sitl_rcin_port", "sitl_fgview_port")}


def probe_busy_udp_ports() -> Tuple[Optional[Dict[int, str]], str]:
    """用唯讀的 `ss -lun` 讀出「現在已經被 bind 的 UDP 埠」。

    ⚠️⚠️ 這道檢查的**認識論**要講清楚,否則它會變成假綠燈:
      * ss 看到某個埠 ⇒ 那個埠**確實**已經有人 bind ⇒ 我們要用就是撞到 ⇒ 硬失敗。
        (這個方向是可靠的:單向的真陽性。)
      * ss **沒**看到 ⇒ **證明不了任何事**。ArduPilotPlugin 的 socket 走
        SO_REUSEADDR(ArduPilotPlugin.cc:227 / SocketUDP.cc:72-75),而且對方可能
        還沒起來。要真的證明「這個埠是我的」只能實際送探測封包看誰收到,
        那是啟動器的責任(見 ports.json 的 warnings)。
      ⇒ 所以本函式的回傳一律連同「這道檢查到底有沒有跑成功」一起交出去,
        絕不把「沒跑成」當成「檢查通過」。
    """
    exe = shutil.which("ss")
    if exe is None:
        return None, "找不到 `ss`,這道檢查**沒有執行**"
    try:
        proc = subprocess.run([exe, "-lun"], capture_output=True, text=True,
                              timeout=10)
    except Exception as exc:                      # noqa: BLE001
        return None, "`ss -lun` 執行失敗(%s),這道檢查**沒有執行**" % exc
    if proc.returncode != 0:
        return None, ("`ss -lun` exit=%d,這道檢查**沒有執行**"
                      % proc.returncode)
    # ⚠️⚠️ 欄位索引:資料列是
    #        State  Recv-Q  Send-Q  <Local Address:Port>  <Peer Address:Port>
    #      ⇒ 本機位址是 **cols[3]**。標頭列的 "Local Address:Port" 是兩個字,
    #        跟資料列對不齊,不要照標頭數欄位。
    #      (我第一版寫成 cols[4],那是 Peer,值恆為 "0.0.0.0:*",port 不是數字 ⇒
    #       全部被跳過 ⇒ busy 永遠是空的 ⇒ 這道檢查**恆綠**。正是我們在抓的那種 bug。)
    busy: Dict[int, str] = {}
    for line in proc.stdout.splitlines()[1:]:
        cols = line.split()
        if len(cols) < 4:
            continue
        local = cols[3]                            # 例:127.0.0.1:9002 或 [::]:5353
        _, sep, port = local.rpartition(":")
        if not sep or not port.isdigit():
            continue
        busy.setdefault(int(port), local)
    if not busy:
        # ⚠️ 一台在跑的 Linux 一定有 UDP listener(至少 DNS / mDNS)。解析結果是空的
        #    幾乎一定是**我們的剖析壞了**,不是「沒有埠被佔」。這種時候必須報成
        #    「檢查沒跑成」,絕不可以當成通過 —— 恆綠的檢查比沒有檢查更糟。
        return None, ("`ss -lun` 有輸出但一個埠都解析不出來(%d 行),"
                      "判定為剖析失敗,這道檢查**沒有執行**"
                      % len(proc.stdout.splitlines()))
    return busy, "`ss -lun` 讀到 %d 個已 bind 的 UDP 埠" % len(busy)


# ══════════════════════════════════════════════════════════════════════
# 機隊規劃
# ══════════════════════════════════════════════════════════════════════

@dataclass
class Vehicle:
    index: int                # 第 k 架(0-based)
    name: str                 # 世界檔裡的外層 model 名
    profile: Profile
    instance: int             # SITL -I 的值
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    ports: Dict[str, int] = field(default_factory=dict)
    channels: List[dict] = field(default_factory=list)
    # 實際套用 PWM 刻度檢查的通道(側寫 / 推論 / CLI 覆寫三者之一),寫進 ports.json
    # 當作「這道守衛到底有沒有咬到東西」的證據。
    lift_channels: Tuple[int, ...] = ()
    model_sdf: Optional[Path] = None
    # ⚠️ 這台機器上**另一個 session 正在寫** sitl/gazebo/models/quadplane_vtol/model.sdf
    #    (2026-09-09 實測:我第一次 ls 時只有 model.config,兩分鐘後 model.sdf 出現
    #     且當下還是壞的 XML)。本檔會把同一個模型檔解析 3~5 次(main 一次、
    #     build_world 兩次 × 確定性自檢),中途被改掉就會產出「ports.json 記的通道」
    #     與「世界檔裡實際嵌的通道」不一致 —— 而且完全靜默。
    #    ⇒ 記下第一次讀到的 sha1,產完再雜湊一次比對,不一致就硬失敗。
    model_sha1: str = ""
    # z / roll / pitch 的來源(讀回用)。⚠️ 側寫裡的 spawn_z 是「模型還沒生出來時
    #    的佔位值、沒有出處」,而模型檔自己的頂層 <pose> 才是作者算過起落架高度的
    #    那個值(實測:quadplane_vtol 寫 0.25 並註明「起落架最低點 z=-0.25」,
    #    側寫卻是 0.30)⇒ 同一個常數活在兩個地方,改一邊另一邊不會報錯。
    #    所以沒有更明確來源時**以模型檔自己的 pose 為準**,並把來源寫進 ports.json。
    z_source: str = "profile"
    roll_source: str = "profile"
    pitch_source: str = "profile"
    model_own_pose: Optional[Tuple[float, ...]] = None


class _FleetSpecAction(argparse.Action):
    """把 --add 與 --vehicle 收進**同一個**串列,以保留命令列上的先後順序。

    ⚠️ 順序會決定預設隊列位置與預設名字編號,所以順序必須是確定的;
       argparse 用兩個 dest 各自 append 會丟失交錯順序 ⇒ 產出就不確定了。
    """
    def __call__(self, parser, namespace, values, option_string=None):
        specs = getattr(namespace, "fleet_specs", None)
        if specs is None:
            specs = []
            setattr(namespace, "fleet_specs", specs)
        kind = "add" if option_string == "--add" else "vehicle"
        specs.append((kind, values))


def _parse_add(value: str) -> List[dict]:
    """`PROFILE:COUNT` → COUNT 筆只帶 profile 的規格。"""
    parts = value.split(":")
    if len(parts) == 1:
        profile, count = parts[0], 1
    elif len(parts) == 2:
        profile = parts[0]
        try:
            count = int(parts[1])
        except ValueError:
            raise SystemExit("✗ --add %r 的數量不是整數" % value)
    else:
        raise SystemExit("✗ --add 格式是 PROFILE 或 PROFILE:COUNT,收到 %r" % value)
    if not profile:
        raise SystemExit("✗ --add %r 沒給 profile" % value)
    if count < 1:
        raise SystemExit("✗ --add %r 的數量要 ≥ 1" % value)
    return [{"profile": profile} for _ in range(count)]


_VEHICLE_KEYS = {"profile", "instance", "name", "x", "y", "z",
                 "roll", "pitch", "yaw"}


def _parse_vehicle(value: str) -> dict:
    """`profile=X,instance=10,name=vtol1,x=0,y=0,z=0.3,yaw=38` → dict。

    ⚠️ 不認得的 key **硬失敗**。打錯的 key 被安靜忽略正是本專案最痛的失效模式。
    """
    out: dict = {}
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        if "=" not in token:
            raise SystemExit("✗ --vehicle 的 %r 不是 key=value" % token)
        k, v = token.split("=", 1)
        k, v = k.strip(), v.strip()
        if k not in _VEHICLE_KEYS:
            raise SystemExit(
                "✗ --vehicle 不認得的欄位 %r(可用:%s)"
                % (k, ", ".join(sorted(_VEHICLE_KEYS))))
        if k in out:
            raise SystemExit("✗ --vehicle 的 %r 給了兩次" % k)
        out[k] = v
    if "profile" not in out:
        raise SystemExit("✗ --vehicle 至少要有 profile=,收到 %r" % value)
    return out


def _spec_float(spec: dict, key: str) -> float:
    """把 --vehicle 的數值欄位轉成 float。

    ⚠️ 原本直接寫 `float(spec["x"])`,打錯就是一個 raw ValueError traceback,
       使用者看到的是 argparse 之外的內部堆疊。錯是會錯(不是靜默),但訊息不指向
       病灶 —— 統一成跟其他欄位一樣的 SystemExit。
    """
    try:
        return float(spec[key])
    except ValueError:
        raise SystemExit("✗ --vehicle 的 %s=%r 不是數字" % (key, spec[key]))


def plan_fleet(specs: Sequence[dict], instance_base: int, spacing: float,
               hdg_deg: float, spawn_z_override: Optional[float],
               spawn_roll_override: Optional[float],
               min_instance: int) -> List[Vehicle]:
    """把規格清單變成完整的機隊,補齊 instance / 位置 / 朝向 / 埠。

    隊列預設:沿跑道航向的**反方向**排成縱列(與 swarm_sitl_up.sh:360 和舊
    gz_gen_world.py 相同的慣例),間距 spacing。
        bearing = hdg + 180;  east = d·sin(bearing),north = d·cos(bearing)
    """
    # ENU 的 yaw 從 +X(東)逆時針量,羅盤航向 H 對應 yaw = 90 − H。
    default_yaw = 90.0 - hdg_deg
    bearing = math.radians(hdg_deg + 180.0)

    fleet: List[Vehicle] = []
    used_instances: Dict[int, str] = {}
    used_names: Dict[str, int] = {}
    used_fdm: Dict[int, str] = {}
    prefix_counter: Dict[str, int] = {}
    auto_instance = instance_base

    for k, spec in enumerate(specs):
        prof = resolve_profile(spec["profile"])

        # --- instance:給了就用,沒給就從 instance_base 依序取下一個沒被用過的
        if "instance" in spec:
            try:
                instance = int(spec["instance"])
            except ValueError:
                raise SystemExit("✗ instance %r 不是整數" % spec["instance"])
        else:
            while auto_instance in used_instances:
                auto_instance += 1
            instance = auto_instance
            auto_instance += 1

        # --- 名字
        if "name" in spec:
            name = spec["name"]
        else:
            n = prefix_counter.get(prof.name_prefix, 0) + 1
            prefix_counter[prof.name_prefix] = n
            name = "%s%d" % (prof.name_prefix, n)

        # ⚠️⚠️⚠️ instance 下限:低號碼的埠**現在**就被另一個 session 佔著,
        #    而撞埠在執行期是完全靜默的(SO_REUSEADDR)。見 MIN_INSTANCE_DEFAULT。
        if instance < min_instance:
            raise SystemExit(
                "✗ %s 的 instance %d 低於下限 %d。\n"
                "  低號碼推出來的埠(fdm %d / sitl_in %d / irlock %d / rcin %d)"
                "與這台機器上既有的航次重疊,而撞埠**完全靜默**:\n"
                "  SO_REUSEADDR 讓兩邊都 bind 成功,後 bind 的收全部封包、"
                "先 bind 的收零,無錯誤、無 log、ss 也看不出來\n"
                "  (ArduPilotPlugin.cc:227 / SocketUDP.cc:72-75)。\n"
                "  ⇒ 真的要用低號碼請明講:--min-instance %d(或 GZ_VTOL_MIN_INSTANCE)。"
                % (name, instance, min_instance,
                   FDM_PORT_BASE + PORT_STRIDE * instance,
                   SITL_SIM_IN_BASE + PORT_STRIDE * instance,
                   IRLOCK_PORT_BASE + PORT_STRIDE * instance,
                   SITL_RCIN_PORT_BASE + PORT_STRIDE * instance,
                   instance))

        # --- 位置:沒給就走縱列
        d = k * spacing
        x = _spec_float(spec, "x") if "x" in spec else d * math.sin(bearing)
        y = _spec_float(spec, "y") if "y" in spec else d * math.cos(bearing)
        # ⚠️ z/roll/pitch 的來源要記下來:只有「還在走 profile 佔位值」的那些,
        #    稍後才會被模型檔自己的 <pose> 取代(見 Vehicle.z_source 的註解)。
        if "z" in spec:
            z, z_src = _spec_float(spec, "z"), "vehicle-spec"
        elif spawn_z_override is not None:
            z, z_src = spawn_z_override, "cli/env spawn_z"
        else:
            z, z_src = prof.spawn_z, "profile"

        if "roll" in spec:
            roll, roll_src = _spec_float(spec, "roll"), "vehicle-spec"
        elif spawn_roll_override is not None:
            roll, roll_src = spawn_roll_override, "cli/env spawn_roll"
        else:
            roll, roll_src = prof.spawn_roll_deg, "profile"

        if "pitch" in spec:
            pitch, pitch_src = _spec_float(spec, "pitch"), "vehicle-spec"
        else:
            pitch, pitch_src = prof.spawn_pitch_deg, "profile"
        yaw = _spec_float(spec, "yaw") if "yaw" in spec else default_yaw

        ports = ports_for_instance(instance)

        # ── 唯一性:三道硬失敗 ────────────────────────────────────────
        # ⚠️⚠️ fdm 埠重複是**執行期完全隱形**的(SO_REUSEADDR,見檔頭【坑 2】):
        #    後 bind 的收到全部封包、先 bind 的收到零,無錯誤、無 log,ss 也看不出來。
        #    這裡是整條鏈上唯一能擋下它的地方。
        if instance in used_instances:
            raise SystemExit("✗ instance %d 被 %s 與 %s 同時使用"
                             % (instance, used_instances[instance], name))
        if ports["fdm_port_in"] in used_fdm:
            raise SystemExit(
                "✗ fdm 埠 %d 重複(%s 與 %s)—— SO_REUSEADDR 會讓兩邊都 bind 成功"
                "且完全靜默,一定要在這裡擋下"
                % (ports["fdm_port_in"], used_fdm[ports["fdm_port_in"]], name))
        if name in used_names:
            # gz 對同名 model 的行為是後者載不進去,而且很容易被淹在 warning 裡。
            raise SystemExit("✗ model 名稱 %r 重複(第 %d 架與第 %d 架)"
                             % (name, used_names[name], k))
        used_instances[instance] = name
        used_fdm[ports["fdm_port_in"]] = name
        used_names[name] = k

        fleet.append(Vehicle(index=k, name=name, profile=prof, instance=instance,
                             x=x, y=y, z=z, roll_deg=roll, pitch_deg=pitch,
                             yaw_deg=yaw, ports=ports, z_source=z_src,
                             roll_source=roll_src, pitch_source=pitch_src))
    if not fleet:
        raise SystemExit("✗ 機隊是空的:至少要一個 --add 或 --vehicle")
    return fleet


# ══════════════════════════════════════════════════════════════════════
# 模型檔搬運 + 讀回式驗證
# ══════════════════════════════════════════════════════════════════════

def _parse_keep_comments(path: Path) -> ET.ElementTree:
    """解析 SDF 但**保留註解**。

    上游模型檔的註解(例如 iris 那段 "multiplier = 838 max rpm" 的推導)是有價值的,
    搬進世界檔時不該丟掉。ET 預設會吃掉註解,要自己換 TreeBuilder。
    (insert_comments 自 Python 3.8 起可用;WSL 這台是 3.10.12。)
    """
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    try:
        return ET.parse(str(path), parser=parser)
    except ET.ParseError as exc:
        raise SystemExit("✗ %s 不是合法的 XML:%s" % (path, exc))


def find_model_sdf(model_dir: str, model_path: Sequence[Path]) -> Path:
    """在 model_path 上逐段找 `<dir>/<model_dir>/model.sdf`。"""
    tried = []
    for base in model_path:
        cand = base / model_dir / "model.sdf"
        tried.append(str(cand))
        if cand.is_file():
            return cand
    raise SystemExit(
        "✗ 找不到模型 %r 的 model.sdf。找過:\n    %s\n"
        "  ⇒ 若模型還沒產出(VTOL 由另一個 session 負責),先等它;\n"
        "     若目錄名不同,直接把目錄名當 profile 傳進來,或加 --model-path。"
        % (model_dir, "\n    ".join(tried)))


def _parse_pose_text(text: str, path: Path) -> Tuple[float, ...]:
    """把 <pose> 的六個數字讀出來。⚠️ 只接受剛好六個,少一個就別猜。"""
    parts = (text or "").split()
    if len(parts) != 6:
        raise SystemExit("✗ %s 的 <pose> 不是六個數字:%r" % (path, text))
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        raise SystemExit("✗ %s 的 <pose> 含非數字:%r" % (path, text))


def inspect_model_body(path: Path) -> Tuple[ET.Element, ET.Element, List[dict],
                                            Optional[Tuple[float, ...]]]:
    """讀出模型檔的 <model>、它的 ArduPilotPlugin、逐通道的刻度,以及它自己的 <pose>。

    ⚠️ 這是**讀回**,不是改寫。本檔刻意不去「順手修好」別人模型裡的數字:
       靜默改寫比靜默失敗更難查。發現不對就報出來,由模型作者去修。
    """
    root = _parse_keep_comments(path).getroot()
    if root.tag != "sdf":
        raise SystemExit("✗ %s 的根節點是 <%s>,不是 <sdf>" % (path, root.tag))
    models = [c for c in root if c.tag == "model"]
    if len(models) != 1:
        raise SystemExit(
            "✗ %s 的頂層 <model> 有 %d 個(預期剛好 1)。本產生器一次搬一台機體。"
            % (path, len(models)))
    body = models[0]

    plugins = [p for p in body.iter("plugin")
               if p.get("filename") == "ArduPilotPlugin"
               or p.get("name") == "ArduPilotPlugin"]
    if len(plugins) != 1:
        raise SystemExit(
            "✗ %s 裡的 ArduPilotPlugin 有 %d 個(預期剛好 1)。\n"
            "  一個外層 model 帶一份 plugin —— 這是 swarm3.sdf 實測會動的寫法。"
            % (path, len(plugins)))
    plugin = plugins[0]

    ports = plugin.findall("fdm_port_in")
    if len(ports) != 1:
        raise SystemExit(
            "✗ %s 的 <fdm_port_in> 出現 %d 次(預期 1)—— 模型版面變了,不敢亂改埠"
            % (path, len(ports)))

    # ⚠️ <fdm_port_out> 已 deprecated(ArduPilotPlugin.cc:1280-1283),留著只會
    #    讓人以為它有用。plugin 是 sendto 回封包來源(:2031),不看這個欄位。
    if plugin.find("fdm_port_out") is not None:
        raise SystemExit(
            "✗ %s 還帶著已 deprecated 的 <fdm_port_out>(ArduPilotPlugin.cc:1280-1283)。"
            "留著會誤導:plugin 實際是回傳給封包來源,不讀這個欄位。" % path)

    channels: List[dict] = []
    # ⚠️⚠️⚠️ 這裡原本寫「同一個 channel 出現兩次就硬失敗」——**那是錯的**,而且是
    #   個假守衛:它擋掉的是合法寫法,擋不到任何真的 bug。
    #   實證(唯讀讀原始碼):ArduPilotPlugin 的 controls 是一個 **vector**,
    #     ArduPilotPlugin.cc:82-99   class Control 每個自己帶一個 channel 與一個 joint
    #     ArduPilotPlugin.cc:539-573 逐個 <control> 解析,channel 只是屬性,不做唯一性檢查
    #     ArduPilotPlugin.cc:801     controls.push_back(control)
    #     ArduPilotPlugin.cc:1648-1652 逐 i 取 _pwm[controls[i].channel]
    #   ⇒ 多個 control 共用同一個 PWM channel 是**完全合法**的,而且是兩塊副翼 /
    #     兩片 elevon / V 尾這類機構的正常做法。
    #   ⇒ 實害:本 repo 真正要飛的 sitl/gazebo/models/quadplane_vtol/model.sdf 就是
    #     channel 0 掛 aileron_left_joint 與 aileron_right_joint 兩個 control。
    #     舊版對它**整個產不出世界檔**(2026-09-09 實測硬失敗)。
    #   ⇒ 改成:允許共用,但要求共用同一 channel 的 control 其 servo_min/servo_max
    #     **必須一致**。不一致才是真的 bug(同一個 PWM 值餵給兩個刻度不同的 joint,
    #     兩邊行程對不起來,而且沒有任何告警)。
    by_channel: Dict[int, dict] = {}
    for ctrl in plugin.findall("control"):
        raw = ctrl.get("channel")
        if raw is None:
            raise SystemExit("✗ %s 有 <control> 沒有 channel 屬性" % path)
        try:
            ch = int(raw)
        except ValueError:
            raise SystemExit("✗ %s 的 control channel=%r 不是整數" % (path, raw))
        # 【出處】control channel 上限實際是 16(0-15)。
        if not (0 <= ch <= MAX_CONTROL_CHANNEL):
            raise SystemExit("✗ %s 的 control channel=%d 超出 0..%d"
                             % (path, ch, MAX_CONTROL_CHANNEL))
        joint = (ctrl.findtext("jointName") or "").strip()

        def _int_or_none(tag: str, _ch=ch):
            t = ctrl.findtext(tag)
            if t is None or t.strip() == "":
                return None
            try:
                return int(float(t.strip()))
            except ValueError:
                raise SystemExit("✗ %s channel=%d 的 <%s>=%r 不是數字"
                                 % (path, _ch, tag, t))

        rec = {
            "channel": ch,
            "joint": joint,
            "type": (ctrl.findtext("type") or "").strip(),
            "servo_min": _int_or_none("servo_min"),
            "servo_max": _int_or_none("servo_max"),
        }
        prev = by_channel.get(ch)
        if prev is not None and (prev["servo_min"] != rec["servo_min"]
                                 or prev["servo_max"] != rec["servo_max"]):
            raise SystemExit(
                "✗ %s 的 channel=%d 被 %s 與 %s 共用,但 PWM 刻度不一致"
                "(%s/%s vs %s/%s)。\n"
                "  共用 channel 本身合法(ArduPilotPlugin.cc:1648-1652 逐 control 取"
                " _pwm[channel]),但同一個 PWM 值餵給兩個刻度不同的 joint,"
                "兩邊行程對不起來且完全沒有告警。"
                % (path, ch, prev["joint"], joint,
                   prev["servo_min"], prev["servo_max"],
                   rec["servo_min"], rec["servo_max"]))
        by_channel.setdefault(ch, rec)
        channels.append(rec)
    # ⚠️ 排序鍵要含 joint:同一個 channel 有多個 control 時,只用 channel 排序在
    #    CPython 上雖然穩定,但「輸出的確定性靠 sort 的實作細節」不是我們要的保證。
    channels.sort(key=lambda c: (c["channel"], c["joint"]))
    if not channels:
        raise SystemExit("✗ %s 的 ArduPilotPlugin 一個 <control> 都沒有" % path)

    # 模型自己的頂層 <pose>:讀出來(不是拿來嵌的,見 _build_vehicle_model)。
    own_poses = [c for c in body if c.tag == "pose"]
    if len(own_poses) > 1:
        raise SystemExit("✗ %s 的 <model> 有 %d 個頂層 <pose>" % (path, len(own_poses)))
    own_pose = None
    if own_poses:
        # ⚠️ degrees 屬性:模型若用弧度而我們當角度用就會歪 57 倍。SDF 的 <pose>
        #    預設是**弧度**;只有 degrees="true" 才是角度。
        #    我們只拿 z(平移,單位無關)當預設值,roll/pitch 只在模型明示 degrees
        #    時才拿來用,其餘情況維持側寫值並在報告裡說明。
        own_pose = _parse_pose_text(own_poses[0].text or "", path)
        if (own_poses[0].get("degrees") or "").lower() not in ("true", "1"):
            own_pose = own_pose[:3] + (float("nan"),) * 3
    return body, plugin, channels, own_pose


def lift_channels_for(veh: Vehicle,
                      override: Optional[Tuple[int, ...]]) -> Tuple[int, ...]:
    """決定這一架的哪些 control channel 算「升力馬達」。

    ⚠️⚠️ 這個函式存在的唯一理由:**不讓【坑 7】的守衛靜默消失**。
       側寫是用 profile 名字查的,而 VTOL 模型的目錄名由另一個 session 決定
       —— 名字對不上就走 resolve_profile() 的 fallback,fallback 的
       lift_channels 是空的 ⇒ 守衛不會報錯,只會**什麼都不檢查**。
       那正是本專案記過的「說有鎖、其實測試收集 0 項」那一類失效。

    ⇒ 規則(由強到弱):
        1) CLI 的 lift_channels 覆寫(含明確關掉)。
        2) profile 內建側寫。
        3) 推論:模型裡出現 channel ≥ 4 就當它是 QuadPlane,channel ≥ 4 全部算升力馬達。
           依據是通道映射(已逐檔驗證):ArduPlane 只用 SERVO1-4 ⇒ channel 0-3;
           純多旋翼 iris 也只有 channel 0-3。**只有** QuadPlane 才會用到 channel 4+
           (升力馬達 = SERVO5-8,SIM_QuadPlane 的 motor_offset 預設 4)。
           推論命中時會印出來,不做無聲的事。
    """
    if override is not None:
        return override
    if veh.profile.lift_channels:
        return veh.profile.lift_channels
    inferred = tuple(sorted(c["channel"] for c in veh.channels if c["channel"] >= 4))
    if inferred:
        print("[lift] %s(profile=%s 無內建側寫)偵測到 channel %s ⇒ 依通道映射推論為"
              "升力馬達,套用 PWM 刻度檢查"
              % (veh.name, veh.profile.key,
                 ",".join(str(c) for c in inferred)), file=sys.stderr)
    return inferred


def check_lift_pwm(veh: Vehicle, lift_channels: Sequence[int],
                   lift_min: int, lift_max: int) -> None:
    """【坑 7】升力馬達的 servo_min/servo_max 必須與 Q_M_PWM_MIN/MAX 對齊。

    不對齊的後果不是報錯,是 ArduPilotPlugin.cc:1676-1678 把
    raw_cmd=(pwm−pwm_min)/(pwm_max−pwm_min) clamp 到 [0,1] ⇒
    底部 12.5% 全夾成 0、頂部 12.5% 全夾成 1,四顆一起損失 25% 行程。
    這在飛行紀錄上看起來只是「推力不太夠」,不會有任何告警。
    """
    if not lift_channels:
        return
    # ⚠️⚠️ 原本是 `by_ch = {c["channel"]: c for c in veh.channels}` —— 一個 channel
    #    掛多個 control 時 dict 只會留**最後一個**,前面那些就靜靜地沒被檢查到。
    #    這正是「守衛看起來有跑、其實只檢查了一半」那一類。改成逐 control 全查。
    checked = 0
    for ch in lift_channels:
        matched = [c for c in veh.channels if c["channel"] == ch]
        if not matched:
            raise SystemExit(
                "✗ %s(%s):側寫說 channel %d 是升力馬達,但模型裡沒有這個 control。\n"
                "  QuadPlane 升力馬達固定在 SERVO5-8 ⇒ channel 4/5/6/7"
                "(SIM_QuadPlane 的 motor_offset 預設 4)。"
                % (veh.name, veh.profile.model_dir, ch))
        for c in matched:                     # 逐個查,不是拿一個當代表
            checked += 1
            if c["servo_min"] == lift_min and c["servo_max"] == lift_max:
                continue
            raise SystemExit(
                "✗ %s(%s)channel %d(%s)的 PWM 刻度是 %s/%s,應為 %d/%d。\n"
                "  這不是風格問題:ArduPilotPlugin.cc:1676-1678 的\n"
                "      raw_cmd = clamp((pwm − servo_min)/(servo_max − servo_min), 0, 1)\n"
                "  會把 Q_M_PWM_MIN/MAX=%d/%d 的底部與頂部各 12.5%% 夾掉,\n"
                "  四顆一起損失 25%% 行程,而且完全沒有告警。\n"
                "  ⇒ 請模型作者把 servo_min/servo_max 改成 %d/%d(本檔不擅自改別人的模型)。\n"
                "  ⚠️ 另注意 Q_M_PWM_TYPE 預設 NORMAL ⇒ SERVOn_MIN/MAX 對馬達輸出不生效。"
                % (veh.name, veh.profile.model_dir, ch, c["joint"],
                   c["servo_min"], c["servo_max"], lift_min, lift_max,
                   lift_min, lift_max, lift_min, lift_max))
    # ⚠️ 「這道守衛真的咬到東西了嗎」要有證據,不能只看它沒噴錯:
    #    checked==0 表示 lift_channels 非空卻一個 control 都沒比對到 —— 上面的
    #    not matched 已經擋掉了,所以走到這裡還是 0 只可能是內部錯誤。大聲講。
    if checked == 0:
        raise SystemExit(
            "✗ 內部錯誤:%s 宣告了升力通道 %s 卻一個 control 都沒檢查到。"
            % (veh.name, list(lift_channels)))
    print("[lift] %s:PWM 刻度守衛檢查了 %d 個 control(channel %s),全部 %d/%d"
          % (veh.name, checked, ",".join(str(c) for c in lift_channels),
             lift_min, lift_max), file=sys.stderr)


# ══════════════════════════════════════════════════════════════════════
# 世界檔組裝
# ══════════════════════════════════════════════════════════════════════

def _f(v: float, nd: int = 4) -> str:
    """定寬浮點格式化 —— 確定性的關鍵。

    ⚠️ 順便把 "-0.0000" 正規化成 "0.0000":math.sin(pi) 之類會產生 −1e−16,
       格式化後變 "-0.0000",而同一個輸入在不同 libm 上可能給 "+0.0000"
       ⇒ md5 不穩。本專案踩過「產生檔每次重跑 md5 都不同」的假 diff 坑
       (記憶 export-must-be-deterministic)。
    """
    s = "%.*f" % (nd, v)
    if set(s) <= set("-0."):
        s = "%.*f" % (nd, 0.0)
    return s


def _comment(text: str) -> ET.Element:
    """建 XML 註解,並擋掉會讓 XML 失效的內容。

    XML 註解裡不能出現 "--",也不能以 "-" 結尾。⚠️ 這很容易踩到:我們的註解常想寫
    命令列旗標(例如 serial0 那個不等待的選項),寫下去整個世界檔就解析失敗,
    而錯誤訊息只會說 XML 壞了,不會說是註解的問題。
    """
    if "--" in text:
        raise SystemExit("✗ 內部錯誤:XML 註解不可含 '--':%r" % text)
    if text.rstrip().endswith("-"):
        raise SystemExit("✗ 內部錯誤:XML 註解不可以 '-' 結尾:%r" % text)
    return ET.Comment(" " + text.strip() + " ")


def _sub(parent: ET.Element, tag: str, text=None, **attrs) -> ET.Element:
    e = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e


def build_world(world_name: str, fleet: Sequence[Vehicle], *,
                origin_lat: float, origin_lon: float, origin_elev: float,
                hdg_deg: float, step_size: float, rtf: float,
                ground_size: float, with_runway: bool,
                fdm_addr: str) -> ET.Element:
    root = ET.Element("sdf", {"version": "1.9"})

    root.append(_comment(
        "本檔由 scripts/wsl/gz_vtol_world.py 產生。⚠️ 不要手改 —— 改產生器。"))
    root.append(_comment(
        "結構:每一架是一個外層 <model>,各自帶一份 ArduPilotPlugin,"
        "內文只 include 最底層的裸模型。這是 ~/swarm/gazebo/swarm3.sdf 實測會動的寫法"
        "(四機、四份 plugin、四個都 include 同一個 iris_with_standoffs、RTF 0.236)。"))
    root.append(_comment(
        "⚠️⚠️ fdm_port_in 由 SITL instance 推出:9002 + 10*instance。"
        "出處 AP_HAL_SITL/SITL_cmdline.cpp:258-259 與 :444-449。"
        "改任何一邊都要兩邊一起改,對照表在同名的 .ports.json。"))
    root.append(_comment(
        "⚠️⚠️ 埠寫錯**不會報錯**:ArduPilotPlugin 的 socket 走 SO_REUSEADDR"
        "(ArduPilotPlugin.cc:227 / SocketUDP.cc:72-75),重複 bind 兩邊都成功,"
        "後 bind 的收全部、先 bind 的收零,ss 完全看不出來。"))

    world = _sub(root, "world", name=world_name)

    # ── 物理 ──────────────────────────────────────────────────────────
    world.append(_comment(
        "步長必須 1 ms(swarm3.sdf 實測):改 2.5 ms 可把 RTF 從 0.15 拉到 0.38,"
        "但陀螺取樣率掉到讓 ArduCopter 速率環進滾轉極限環"
        "(max|R| 386 deg/s vs 指令 89、馬達混控 12% 取樣飽和),滿油門仍一路沉到地面;"
        "回 1 ms 後 max|R| 55、飽和 0%。這是正確性換速度,不要為了跑快改回去。"))
    phys = _sub(world, "physics", name="1ms", type="ignored")
    _sub(phys, "max_step_size", _f(step_size, 6))
    _sub(phys, "real_time_factor", _f(rtf, 4))

    for fn, cls in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ):
        _sub(world, "plugin", filename=fn, name=cls)

    scene = _sub(world, "scene")
    _sub(scene, "ambient", "1.0 1.0 1.0")
    _sub(scene, "background", "0.5 0.7 0.9")
    _sub(scene, "sky")

    # ── 地理原點 ──────────────────────────────────────────────────────
    world.append(_comment(
        "原點必須與 SITL 的 home 參數一致(SWARM_LOC=\"25.0777,121.2325,32,52\","
        "由 swarm_sitl_up.sh 產生的 swarm_state.env 而來),否則 Gazebo 的地理座標與"
        "飛控 HOME 對不上,所有 lat/lon 相關驗收失去意義。"))
    world.append(_comment(
        "⚠️⚠️ 未解的基準問題:下面那塊地面在世界 z=0,而這裡的 elevation 是 %s m "
        "⇒ Gazebo 裡的「海平面」其實是 %s m AMSL。而打擊線的紅心釘死 0 m AMSL。"
        "本階段(鏈路驗證)不受影響,但做打擊驗收前必須先處理這 %s m 落差,"
        "否則會重演「殘餘高度恆定」那一類的地形落差 bug。"
        % (_f(origin_elev, 1), _f(origin_elev, 1), _f(origin_elev, 1))))
    world.append(_comment(
        "heading_deg 保持 0:它旋轉的是**世界座標框**,不是飛機航向。"
        "跑道與機體的朝向一律用各自 pose 的 yaw 表示(上游 zephyr_runway.sdf 也是這樣)。"))
    sph = _sub(world, "spherical_coordinates")
    _sub(sph, "surface_model", "EARTH_WGS84")
    _sub(sph, "world_frame_orientation", "ENU")
    _sub(sph, "latitude_deg", "%.6f" % origin_lat)
    _sub(sph, "longitude_deg", "%.6f" % origin_lon)
    _sub(sph, "elevation", _f(origin_elev, 3))
    _sub(sph, "heading_deg", _f(SPHERICAL_HEADING_DEG, 1))

    light = _sub(world, "light", type="directional", name="sun")
    _sub(light, "cast_shadows", "true")
    _sub(light, "pose", "0 0 100 0 0 0")
    _sub(light, "diffuse", "0.9 0.9 0.9 1")
    _sub(light, "specular", "0.3 0.3 0.3 1")
    _sub(light, "direction", "-0.5 0.3 -0.9")

    # ── 地面(內嵌) ──────────────────────────────────────────────────
    world.append(_comment(
        "⚠️ 地面必須**內嵌**,不可用 <include>model://ground_plane:那個模型本機沒有,"
        "gz 預設會去 Fuel 線上抓,離線環境直接 Error Code 14 讓整個世界起不來"
        "(實測 2026-08-11)。"))
    world.append(_comment(
        "⚠️ 而且地面一定要有:上游模板 zephyr_runway.sdf 只有跑道盤、沒有地面,"
        "跑道一旦沒蓋住機體起點,飛機就直接掉下去,而且 Gazebo 不會報錯。"
        "尺寸 %s m 是因為任務要飛 60 km。" % _f(ground_size, 0)))
    gp = _sub(world, "model", name="ground_plane")
    _sub(gp, "static", "true")
    gl = _sub(gp, "link", name="link")
    for kind, mat in (("collision", False), ("visual", True)):
        node = _sub(gl, kind, name=kind)
        geo = _sub(node, "geometry")
        plane = _sub(geo, "plane")
        _sub(plane, "normal", "0 0 1")
        _sub(plane, "size", "%s %s" % (_f(ground_size, 0), _f(ground_size, 0)))
        if mat:
            m = _sub(node, "material")
            _sub(m, "ambient", "0.30 0.42 0.30 1")
            _sub(m, "diffuse", "0.30 0.42 0.30 1")
            _sub(m, "specular", "0.1 0.1 0.1 1")

    # ── 原點三軸(目視基準) ──────────────────────────────────────────
    world.append(_comment(
        "原點三軸(紅=X/東 綠=Y/北 藍=Z/上),用來目視確認朝向。純視覺,不參與物理。"))
    axes = _sub(world, "model", name="axes")
    _sub(axes, "static", "1")
    axl = _sub(axes, "link", name="link")
    for vname, pose, size, rgb in (
        ("r", "5 0 0.1 0 0 0", "10 0.01 0.01", "1 0 0"),
        ("g", "0 5 0.1 0 0 0", "0.01 10 0.01", "0 1 0"),
        ("b", "0 0 5.1 0 0 0", "0.01 0.01 10", "0 0 1"),
    ):
        vis = _sub(axl, "visual", name=vname)
        _sub(vis, "cast_shadows", "0")
        _sub(vis, "pose", pose)
        box = _sub(_sub(vis, "geometry"), "box")
        _sub(box, "size", size)
        mat = _sub(vis, "material")
        for tag in ("ambient", "diffuse", "emissive"):
            _sub(mat, tag, "%s 0.8" % rgb)

    # ── 跑道 ──────────────────────────────────────────────────────────
    if with_runway:
        # ENU 的 yaw 從 +X(東)逆時針量,羅盤航向 H ⇒ yaw = 90 − H。
        yaw = 90.0 - hdg_deg
        # ⚠️ **旋轉跑道時位移必須一起轉**,否則跑道會從機體底下滑開
        #    (實測 2026-08-11:忘了轉位移 ⇒ 原點下面變空的 ⇒
        #     SIMSTATE 抓到 roll=3.141 / pitch=1.544,翻覆且機頭朝天)。
        d = math.radians(yaw - RUNWAY_TEMPLATE_YAW)
        rx = RUNWAY_TEMPLATE_X * math.cos(d) - RUNWAY_TEMPLATE_Y * math.sin(d)
        ry = RUNWAY_TEMPLATE_X * math.sin(d) + RUNWAY_TEMPLATE_Y * math.cos(d)
        world.append(_comment(
            "跑道:上游模板的 pose 是 (%s, %s) 配 yaw %s;航向 %s° ⇒ yaw %s°,"
            "位移跟著轉 %s° 後是 (%s, %s)。VTOL 不需要跑道,留著是視覺尺規"
            "與混機種時給固定翼用。"
            % (_f(RUNWAY_TEMPLATE_X, 0), _f(RUNWAY_TEMPLATE_Y, 0),
               _f(RUNWAY_TEMPLATE_YAW, 0), _f(hdg_deg, 0), _f(yaw, 0),
               _f(math.degrees(d), 0), _f(rx, 1), _f(ry, 1))))
        inc = _sub(world, "include")
        _sub(inc, "uri", "model://runway")
        pose = _sub(inc, "pose", "%s %s 0 0 0 %s" % (_f(rx, 3), _f(ry, 3), _f(yaw, 3)))
        pose.set("degrees", "true")

    # ── 機體 ──────────────────────────────────────────────────────────
    for veh in fleet:
        world.append(_comment(
            "第 %d 架:%s  |  profile=%s  model=%s  |  SITL instance %d  "
            "|  fdm_port_in %d(SITL 送過來)  |  SITL 收回 %d  |  irlock %d"
            % (veh.index, veh.name, veh.profile.key, veh.profile.model_dir,
               veh.instance, veh.ports["fdm_port_in"],
               veh.ports["sitl_sim_in"], veh.ports["irlock_port"])))
        world.append(_build_vehicle_model(veh, fdm_addr))

    return root


def _build_vehicle_model(veh: Vehicle, fdm_addr: str) -> ET.Element:
    """把一架的 model 內文原樣搬進來,只改外層名字與 fdm 埠。"""
    assert veh.model_sdf is not None
    body, plugin, _channels, _own_pose = inspect_model_body(veh.model_sdf)

    # ⚠️ 只允許 name 這一個屬性。像 canonical_link 這種屬性換了外層名就可能失效,
    #    與其安靜地丟掉,不如硬失敗。
    extra_attrs = {k: v for k, v in body.attrib.items() if k != "name"}
    if extra_attrs:
        raise SystemExit(
            "✗ %s 的 <model> 帶了本產生器沒處理的屬性 %r。\n"
            "  外層 model 會被改名,這些屬性可能因此失效 —— 請先確認語意再放行。"
            % (veh.model_sdf, extra_attrs))

    outer = ET.Element("model", {"name": veh.name})

    # ⚠️⚠️ pose:原版對「模型自帶頂層 <pose>」直接硬失敗,要求模型作者把它刪掉。
    #    那是**錯的要求**,而且會擋死正事:
    #      (a) model.sdf 必須能單獨被 <include> 載入,自帶 <pose> 是正常且必要的;
    #      (b) 本 repo 真正要飛的 sitl/gazebo/models/quadplane_vtol/model.sdf 就寫了
    #          `<pose>0 0 0.25 0 0 0</pose>`,而且註解說明「起落架最低點在
    #          base_link 座標 z=−0.25,世界檔給 0 會讓機體陷進地面 0.25 m,
    #          lock_step 下第一個 tick 就會吃到巨大接觸力」。2026-09-09 實測:
    #          舊版對這個模型**整個產不出世界檔**。
    #    ⇒ 正解是「丟掉模型自己的 pose、改用世界檔的」,但**要大聲講丟了什麼**,
    #      並且在沒有更明確來源時把它的 z/roll/pitch 當預設值(見 Vehicle.z_source)。
    #      靜靜丟掉才是真正的坑:側寫寫 0.30、模型寫 0.25,兩個地方各有一份,
    #      改一邊另一邊不會報錯。
    dropped = [c for c in list(body) if c.tag == "pose"]
    for c in dropped:
        body.remove(c)

    pose = ET.SubElement(outer, "pose", {"degrees": "true"})
    pose.text = "%s %s %s %s %s %s" % (
        _f(veh.x, 3), _f(veh.y, 3), _f(veh.z, 3),
        _f(veh.roll_deg, 3), _f(veh.pitch_deg, 3), _f(veh.yaw_deg, 3))

    for child in list(body):
        outer.append(child)

    # 改埠。inspect_model_body() 已保證 fdm_port_in 剛好一個。
    new_plugins = [p for p in outer.iter("plugin")
                   if p.get("filename") == "ArduPilotPlugin"
                   or p.get("name") == "ArduPilotPlugin"]
    if len(new_plugins) != 1:                      # 防呆:搬運過程不該改變數量
        raise SystemExit("✗ 內部錯誤:%s 搬運後 ArduPilotPlugin 變成 %d 個"
                         % (veh.name, len(new_plugins)))
    p = new_plugins[0]
    p.find("fdm_port_in").text = str(veh.ports["fdm_port_in"])

    addr = p.find("fdm_addr")
    if addr is None:
        # plugin 的預設就是 127.0.0.1(ArduPilotPlugin.cc:1269-1270),但寫出來比較誠實:
        # 讀世界檔的人不該還要去翻 plugin 原始碼才知道它綁在哪。
        addr = ET.Element("fdm_addr")
        addr.text = fdm_addr
        p.insert(0, addr)
    elif (addr.text or "").strip() != fdm_addr:
        raise SystemExit(
            "✗ %s 的模型寫死 fdm_addr=%r,與本次的 %r 不同。\n"
            "  位址不一致時 plugin 會 bind 在別的介面上,而症狀與埠錯位一模一樣。"
            % (veh.name, (addr.text or "").strip(), fdm_addr))

    # lock_step 的讀回:不是設定它,是確認模型作者選了什麼,並把後果講清楚。
    ls = (p.findtext("lock_step") or "").strip()
    if ls not in ("0", "1"):
        raise SystemExit(
            "✗ %s 的 <lock_step> 是 %r(預期 0 或 1)。"
            "lock_step 決定 world 會不會被一架死掉的機體凍結,不可以含糊。" % (veh.name, ls))
    return outer


# ══════════════════════════════════════════════════════════════════════
# 序列化 / 驗證 / 讀回
# ══════════════════════════════════════════════════════════════════════

def serialize(root: ET.Element) -> str:
    """轉成文字。⚠️ 一律 LF、固定縮排 —— 確定性要求。"""
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    if not body.endswith("\n"):
        body += "\n"
    return '<?xml version="1.0" encoding="utf-8"?>\n' + body


def gz_sdf_check(path: Path, sdf_path: Sequence[Path]) -> None:
    """用 `gz sdf -k` 驗語法 + model:// 解析。驗不過就**硬失敗**。

    ⚠️ 關鍵環境變數是 **SDF_PATH**,不是 GZ_SIM_RESOURCE_PATH。
       2026-09-09 本機實測(libsdformat14.so.14.9.0):
           無環境變數                    → Error Code 14 Unable to find uri[...]   exit=255
           GZ_SIM_RESOURCE_PATH=<models> → 同樣失敗                                 exit=255
           SDF_PATH=<models>             → "Valid."                                 exit=0
       (gz_up.sh 檔頭寫「gz sdf -k 無法解析 model://」是錯的,它只是漏了 SDF_PATH。)

    ⚠️ 只看 exit code,不看 stdout 有沒有 "Valid.":上游對合法檔也會噴一堆
       Warning(例如 iris 的 gz_frame_id 不在 SDF 規格裡),把 warning 當失敗會
       讓這道閘門變成永遠紅燈,然後就會有人把它關掉。
    """
    exe = shutil.which("gz")
    if exe is None:
        # ⚠️ 不提供「找不到 gz 就跳過驗證」的退路。跳過等於產出一個沒驗過的檔,
        #    而本專案的硬性要求就是不接受這種靜默放行。
        raise SystemExit(
            "✗ 找不到 `gz` 執行檔,無法驗證世界檔。\n"
            "  本產生器**拒絕**產出沒驗過的檔案 —— 請在有 gz(Harmonic)的環境跑,"
            "例如 WSL。")
    env = dict(os.environ)
    env["SDF_PATH"] = ":".join(str(p) for p in sdf_path)
    proc = subprocess.run([exe, "sdf", "-k", str(path)],
                          capture_output=True, text=True, env=env)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise SystemExit(
            "✗ gz sdf -k 驗證失敗(exit=%d)。\n"
            "  SDF_PATH=%s\n"
            "  ⇒ 不產出壞檔。上面 Error Code 14 = 某個 model:// 在 SDF_PATH 上找不到。"
            % (proc.returncode, env["SDF_PATH"]))


def readback_world(path: Path, fleet: Sequence[Vehicle]) -> None:
    """從**磁碟上的成品**再讀一次,逐架比對 model 名與 fdm 埠。

    這一步不是多餘的。本專案的硬性要求是「設定值必須讀回驗證」,而且埠寫錯在
    執行期是完全隱形的(SO_REUSEADDR)。讀回是最後一道、也是唯一一道在起飛前
    能發現「我以為我寫了 9102、檔案裡其實是 9002」的檢查。
    """
    root = ET.parse(str(path)).getroot()
    worlds = [c for c in root if c.tag == "world"]
    if len(worlds) != 1:
        raise SystemExit("✗ 讀回:成品的 <world> 有 %d 個" % len(worlds))
    models = {m.get("name"): m for m in worlds[0] if m.tag == "model"}

    for veh in fleet:
        m = models.get(veh.name)
        if m is None:
            raise SystemExit("✗ 讀回:成品裡找不到 model %r" % veh.name)
        plugins = [p for p in m.iter("plugin")
                   if p.get("filename") == "ArduPilotPlugin"
                   or p.get("name") == "ArduPilotPlugin"]
        if len(plugins) != 1:
            raise SystemExit("✗ 讀回:%s 的 ArduPilotPlugin 有 %d 個"
                             % (veh.name, len(plugins)))
        got = (plugins[0].findtext("fdm_port_in") or "").strip()
        want = str(veh.ports["fdm_port_in"])
        if got != want:
            raise SystemExit(
                "✗ 讀回:%s 的 fdm_port_in 是 %r,應為 %r(instance %d)"
                % (veh.name, got, want, veh.instance))

    # 全域唯一性再驗一次(直接看檔案內容,不看我們自己的資料結構)。
    all_ports: Dict[str, str] = {}
    for name, m in models.items():
        for p in m.iter("plugin"):
            if p.get("filename") != "ArduPilotPlugin" and p.get("name") != "ArduPilotPlugin":
                continue
            port = (p.findtext("fdm_port_in") or "").strip()
            if port in all_ports:
                raise SystemExit(
                    "✗ 讀回:fdm 埠 %s 在成品裡被 %s 與 %s 共用 —— SO_REUSEADDR 會"
                    "讓它靜默地只餵其中一架" % (port, all_ports[port], name))
            all_ports[port] = name


# ══════════════════════════════════════════════════════════════════════
# 對照表(啟動器 / coordinator 的單一真相來源)
# ══════════════════════════════════════════════════════════════════════

def build_ports_table(world_name: str, world_path: Path, fleet: Sequence[Vehicle],
                      settings: Settings, *, origin_lat: float, origin_lon: float,
                      origin_elev: float, hdg_deg: float, gz_partition: str,
                      model_path: Sequence[Path], fdm_addr: str,
                      world_md5: str, port_probe: dict,
                      min_instance: int) -> dict:
    """「第 k 架 = instance I = fdm 埠 P」的機器可讀對照表。

    ⚠️ 啟動器與 coordinator **必須讀這一份**,不要各自再算一次埠。
       各算各的正是【坑 1】的成因。
    """
    warnings = [
        "⚠️ fdm 埠重複在執行期完全隱形(SO_REUSEADDR;ArduPilotPlugin.cc:227 / "
        "SocketUDP.cc:72-75):後 bind 的收全部封包、先 bind 的收零,無錯誤無 log,"
        "`ss -lunp` 也看不出來 ⇒ 前置檢查必須實際送探測封包並確認誰收到。",

        "⚠️ 「sim_time 有前進」是必要條件但不充分:從沒收到第一個封包的機體會讓 "
        "arduPilotOnline 維持 false、lock_step 迴圈立刻退出 ⇒ world 照跑、那架靜靜死掉"
        "(ArduPilotPlugin.cc:1204-1213)。要逐機驗:SITL log 有 \"JSON received\" 且"
        "沒有持續的 \"No JSON sensor message\",或逐機讀 MAVLink 姿態在變。",

        "⚠️ 反過來,已上線過的機體斷線時 arduPilotOnline 不會被設回 false ⇒ PreUpdate "
        "卡在 while 迴圈每輪睡 10 ms;gz-sim 的 PreUpdate 是單一更新執行緒 ⇒ "
        "一架死掉會凍結整個 world。因為是 sleep 不是 busy-spin,用 CPU 判別會失效。",

        "⚠️ sitl_tcp_serial0_default 只是韌體的預設算術(5760 + 10*instance)。"
        "instance ≥ 10 時它會落進固定翼 STOT 線既有的 58xx 帶,可能撞上。"
        "啟動器應明確指定 serial0(見 sitl_serial0_note),不要依賴預設。",

        "⚠️ SITL 的 SERIAL0 預設是 tcp:<port>:wait,阻塞在 accept()"
        "(AP_HAL_SITL/UARTDriver.cpp:415-427):沒有 GCS 接上就完全不跑 FDM 迴圈,"
        "表徵與「收不到 FDM」一模一樣。啟動器要嘛每架都掛連線,要嘛明確用不等待的 serial0。",

        "⚠️ 異質機隊:ArduPlane 的 MAVLink 串流預設 POSITION=1 / EXTRA1=1 Hz,"
        "ArduCopter 預設全部 0(GCS_MAVLink_Parameters.cpp:49-59 / :60-71)。"
        "同一份 coordinator 在 plane 上看得到 1 Hz、在 copter 上全黑 ⇒ "
        "連上後必須無條件請求串流並斷言真的收到。串流速率參數已改名為 MAV0_/MAV1_/MAV2_*,"
        "舊的 SR0_/SR1_/SR2_* 會被韌體靜默略過。",

        "⚠️ 這台機器上同時有別人的 Gazebo 在跑 ⇒ 起 gz 一定要設 GZ_PARTITION 隔離。",
    ]

    vehicles = []
    for veh in fleet:
        vehicles.append({
            "index": veh.index,
            "name": veh.name,
            "profile": veh.profile.key,
            "model_dir": veh.profile.model_dir,
            "model_sdf": str(veh.model_sdf),
            "profile_note": veh.profile.note,
            # ── 硬事實:由韌體 instance 算術決定,SDF 必須跟著 ───────────
            "instance": veh.instance,
            "fdm_port_in": veh.ports["fdm_port_in"],
            "sitl_sim_in": veh.ports["sitl_sim_in"],
            "irlock_port": veh.ports["irlock_port"],
            "sitl_rcin_port": veh.ports["sitl_rcin_port"],
            "sitl_fgview_port": veh.ports["sitl_fgview_port"],
            "sitl_tcp_serial0_default": veh.ports["sitl_tcp_serial0_default"],
            "sitl_tcp_serial1_default": veh.ports["sitl_tcp_serial1_default"],
            "sitl_tcp_serial2_default": veh.ports["sitl_tcp_serial2_default"],
            # ── 建議值:啟動器可以不採用,但採用了就要兩邊一致 ───────────
            "sysid_suggested": veh.instance + 1,
            # ⚠️ 參數**名字**:這棵樹用 MAV_SYSID,SYSID_THISMAV 已經不在參數表裡
            #    (ArduPlane/Parameters.cpp:17「SYSID_THISMAV was here」、
            #     :1578 的轉換表 k_param_sysid_this_mav_old → "MAV_SYSID")。
            #    名字寫錯會被韌體**靜默略過**(記憶 sitl-param-renames-silent)
            #    ⇒ coordinator 設完一定要讀回。
            "sysid_param_name": "MAV_SYSID",
            "sitl_args_hard": ["--model", "JSON", "-I%d" % veh.instance],
            # 模型檔的 sha1:世界檔嵌的就是這一份。⚠️ 之後模型被改動而世界檔沒重產,
            # 用這個雜湊比得出來(通道/刻度都在下面的 channels)。
            "model_sha1": veh.model_sha1,
            # ⚠️ `or 0.0` 是為了把 −0.0 正規化掉:math.sin(π) 之類會給 −1e−16,
            #    round 後是 −0.0,json 就寫成 "-0.0" ⇒ 同一組輸入的對照表在不同
            #    libm 上可能長得不一樣,製造假 diff(世界檔那側由 _f() 處理)。
            "pose": {"x": round(veh.x, 4) or 0.0, "y": round(veh.y, 4) or 0.0,
                     "z": round(veh.z, 4) or 0.0,
                     "roll_deg": round(veh.roll_deg, 4) or 0.0,
                     "pitch_deg": round(veh.pitch_deg, 4) or 0.0,
                     "yaw_deg": round(veh.yaw_deg, 4) or 0.0},
            # 每個分量的來源。⚠️ "profile" 代表還在用側寫裡的**無出處佔位值**;
            # "model_own_pose" 代表用的是模型作者算過起落架高度的那個值。
            "pose_source": {"z": veh.z_source, "roll": veh.roll_source,
                            "pitch": veh.pitch_source},
            "model_own_pose_dropped": (
                None if veh.model_own_pose is None
                else [None if v != v else round(v, 6) for v in veh.model_own_pose]),
            # 逐通道 PWM 刻度:讀回值,不是我們設的。【坑 7】的證據就在這裡。
            "channels": veh.channels,
            # 這道守衛實際檢查了哪些通道。空 list = **沒有檢查任何東西**,
            # ⚠️ 對 QuadPlane 來說那就是紅燈,不是綠燈。
            "lift_channels_checked": list(veh.lift_channels),
        })

    return {
        "schema": "aeroplan/gz_vtol_world/ports/1",
        "generator": "scripts/wsl/gz_vtol_world.py",
        "world_name": world_name,
        "world_file": str(world_path),
        "world_md5": world_md5,
        "fdm_addr": fdm_addr,
        "gz_partition": gz_partition,
        "model_path": [str(p) for p in model_path],
        "sdf_path_for_validation": [str(p) for p in model_path],
        "origin": {
            "latitude_deg": origin_lat,
            "longitude_deg": origin_lon,
            "elevation_m": origin_elev,
            "heading_deg": hdg_deg,
            "sitl_home": "%s,%s,%s,%s" % ("%.6f" % origin_lat, "%.6f" % origin_lon,
                                          _f(origin_elev, 1), _f(hdg_deg, 1)),
            "note": ("⚠️ 地面在世界 z=0 而 elevation=%s ⇒ Gazebo 的海平面其實是 "
                     "%s m AMSL;打擊驗收前必須處理這個落差。"
                     % (_f(origin_elev, 1), _f(origin_elev, 1))),
        },
        "port_arithmetic": {
            "source": "AP_HAL_SITL/SITL_cmdline.cpp:250-256(宣告), :435-449(+= instance*10)",
            "fdm_port_in": "9002 + 10 * instance",
            "sitl_sim_in": "9003 + 10 * instance",
            "irlock_port": "9005 + 10 * instance",
            "sitl_rcin_port": "5501 + 10 * instance",
            "sitl_fgview_port": "5503 + 10 * instance",
            "sitl_tcp_serial0_default": "5760 + 10 * instance",
            "note": ("⚠️ 由 SITL instance 推出,不是由 uav 序號推出。"
                     "⚠️ rcin / fgview 這兩個原本漏了;2026-09-09 ss 實測另一個 "
                     "session 的 5501/5511/5521/5531 是真的被佔著的。"),
        },
        "min_instance": min_instance,
        "min_instance_note": (
            "⚠️ instance 低於這個值就硬失敗:低號碼推出來的埠與這台機器上既有的"
            "航次重疊,而撞埠完全靜默(SO_REUSEADDR)。用 --min-instance 覆寫。"),
        "port_probe": port_probe,
        "port_probe_note": (
            "⚠️ 這道檢查是**單向**的:`ss -lun` 看到某個埠 ⇒ 它確實已被 bind ⇒ 我們"
            "會硬失敗;ss **沒**看到 ⇒ 證明不了任何事(SO_REUSEADDR + 對方可能還沒起來)。"
            "ran=false 代表這道檢查根本沒跑成,那不是綠燈。"),
        "sitl_serial0_note": (
            "SERIAL0 預設 tcp:<port>:wait 會阻塞在 accept()"
            "(AP_HAL_SITL/UARTDriver.cpp:415-427),沒 GCS 接上就完全不跑 FDM 迴圈,"
            "表徵與「收不到 FDM」一模一樣。"
            "拿掉這道閘門的寫法是給 SITL 加上不等待的 serial0:`--serial0 tcp:0`"
            "(不帶 :wait;SITL_cmdline.cpp:350-351 的 CMDLINE_SERIAL0 選項、"
            ":506/:516 的處理)。⚠️ 這行字只能放在 JSON 裡:XML 註解不能含連續兩個減號,"
            "所以世界檔的註解寫不出這個旗標,別以為是漏寫。"
            "啟動器必須在腳本裡寫明選了哪一種以及為什麼。"),
        "settings": settings.as_json(),
        "warnings": warnings,
        "vehicles": vehicles,
    }


# ══════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════

def _existing_dirs(spec: str) -> List[Path]:
    """把冒號分隔的搜尋路徑切開。

    ⚠️ 分隔字元是冒號 ⇒ 這個函式在 Windows 上會把 `C:\\Users\\...` 切成
       "C" 與 "\\Users\\..." 兩段垃圾路徑,然後失敗訊息會變成「找不到模型」,
       完全不指向真正的病灶。本檔本來就只能在 WSL 跑(要 gz),所以在入口
       直接擋掉,不要讓人去追一個假症狀。
    """
    if os.name == "nt":
        raise SystemExit(
            "✗ 本檔要在 WSL(Linux)裡跑,不能用 Windows 的 Python。\n"
            "  理由:(1) 需要 `gz`(Harmonic)做 gz sdf 驗證;\n"
            "        (2) 模型搜尋路徑用冒號分隔,Windows 的 C:\\ 會被切爛,\n"
            "            而失敗訊息只會說「找不到模型」,指向錯的地方。\n"
            "  用法:wsl -e bash -lc 'python3 .../scripts/wsl/gz_vtol_world.py ...'")
    out: List[Path] = []
    for token in spec.split(":"):
        token = token.strip()
        if not token:
            continue
        p = Path(os.path.expanduser(token)).resolve()
        if p not in out:
            out.append(p)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="產生多機、可混機種的 Gazebo Harmonic 世界檔(VTOL 群機用)",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("--add", action=_FleetSpecAction, metavar="PROFILE[:COUNT]",
                    help="加 COUNT 台某機種,instance 從 --instance-base 依序取。可重複。")
    ap.add_argument("--vehicle", action=_FleetSpecAction, metavar="K=V,...",
                    help="逐架指定:profile=,instance=,name=,x=,y=,z=,roll=,pitch=,yaw=。可重複。")

    ap.add_argument("--out", default=None,
                    help="世界檔輸出路徑(env GZ_VTOL_OUT)")
    ap.add_argument("--world-name", default=None, help="env GZ_VTOL_WORLD_NAME")
    ap.add_argument("--instance-base", type=int, default=None,
                    help="自動指派 instance 的起點(env GZ_VTOL_INSTANCE_BASE)")
    ap.add_argument("--spacing", type=float, default=None,
                    help="縱列間距 m(env GZ_VTOL_SPACING)")
    ap.add_argument("--hdg", type=float, default=None,
                    help="跑道/隊列航向 deg(env GZ_VTOL_HDG)")
    ap.add_argument("--lat", type=float, default=None, help="env GZ_VTOL_LAT")
    ap.add_argument("--lon", type=float, default=None, help="env GZ_VTOL_LON")
    ap.add_argument("--elevation", type=float, default=None, help="env GZ_VTOL_ELEVATION")
    ap.add_argument("--step-size", type=float, default=None, help="env GZ_VTOL_STEP_SIZE")
    ap.add_argument("--rtf", type=float, default=None, help="env GZ_VTOL_RTF")
    ap.add_argument("--ground-size", type=float, default=None, help="env GZ_VTOL_GROUND_SIZE")
    ap.add_argument("--fdm-addr", default=None, help="env GZ_VTOL_FDM_ADDR")
    ap.add_argument("--gz-partition", default=None, help="env GZ_VTOL_PARTITION")
    ap.add_argument("--model-path", default=None,
                    help="模型搜尋路徑,冒號分隔(env GZ_VTOL_MODEL_PATH)。"
                         "同時也是驗證時的 SDF_PATH。")
    ap.add_argument("--spawn-z", type=float, default=None,
                    help="覆寫所有機體的生成高度(env GZ_VTOL_SPAWN_Z)")
    ap.add_argument("--spawn-roll", type=float, default=None,
                    help="覆寫所有機體的生成 roll(env GZ_VTOL_SPAWN_ROLL)")
    ap.add_argument("--lift-pwm-min", type=int, default=None, help="env GZ_VTOL_LIFT_PWM_MIN")
    ap.add_argument("--lift-pwm-max", type=int, default=None, help="env GZ_VTOL_LIFT_PWM_MAX")
    ap.add_argument("--lift-channels", default=None,
                    help="覆寫升力馬達通道,逗號分隔(例 4,5,6,7);"
                         "'none' = 明確關閉 PWM 刻度檢查(env GZ_VTOL_LIFT_CHANNELS)")

    ap.add_argument("--min-instance", type=int, default=None,
                    help="instance 下限,低於就硬失敗(env GZ_VTOL_MIN_INSTANCE)。"
                         "預設 %d:低號碼推出來的埠與這台機器上既有的航次重疊,"
                         "而撞埠是完全靜默的。" % MIN_INSTANCE_DEFAULT)
    ap.add_argument("--allow-busy-ports", action="store_true",
                    help="即使 `ss -lun` 已經看到規劃中的 UDP 埠被 bind 也照樣產出"
                         "(只降成警告)。⚠️ 幾乎不該用。")
    ap.add_argument("--no-runway", dest="runway", action="store_false", default=True,
                    help="不放跑道(VTOL 用不到,只是視覺尺規;env GZ_VTOL_RUNWAY)")
    ap.add_argument("--dry-run", action="store_true",
                    help="只規劃並印對照表,不寫檔、不驗證")
    ap.add_argument("--no-determinism-check", dest="determinism", action="store_false",
                    default=True, help="關掉「重建一次比對逐字相同」的自檢(不建議)")

    args = ap.parse_args(argv)

    cfg = Settings()
    home = Path(os.path.expanduser("~"))
    repo_root = Path(__file__).resolve().parents[2]   # scripts/wsl/x.py → repo 根
    default_model_path = ":".join([
        # 我們自己的 VTOL 模型。⚠️ 2026-09-09 對照:VTOL 模型那個 session 把它放在
        #    <repo>/sitl/gazebo/models/quadplane_vtol/。這條要排在最前面,
        #    這樣 repo 內的版本永遠蓋過任何散落在家目錄的舊複本。
        str(repo_root / "sitl" / "gazebo" / "models"),
        str(Path(__file__).resolve().parent / "gazebo" / "models"),  # scripts/wsl/gazebo/models(若有)
        str(home / "vtolswarm" / "models"),           # 家目錄的工作區(選用)
        str(home / "ardupilot_gazebo" / "models"),    # 上游 iris / zephyr / runway
    ])

    out = Path(os.path.expanduser(cfg.get(
        "out", args.out, "GZ_VTOL_OUT",
        str(home / "vtolswarm" / "worlds" / "taoyuan_vtol.sdf"), str)))
    world_name = cfg.get("world_name", args.world_name, "GZ_VTOL_WORLD_NAME",
                         "taoyuan_vtol", str)
    instance_base = cfg.get("instance_base", args.instance_base,
                            "GZ_VTOL_INSTANCE_BASE", 10, int)
    spacing = cfg.get("spacing", args.spacing, "GZ_VTOL_SPACING",
                      DEFAULT_SPACING_M, float)
    hdg = cfg.get("hdg", args.hdg, "GZ_VTOL_HDG", ORIGIN_HDG_DEG, float)
    lat = cfg.get("lat", args.lat, "GZ_VTOL_LAT", ORIGIN_LAT, float)
    lon = cfg.get("lon", args.lon, "GZ_VTOL_LON", ORIGIN_LON, float)
    elev = cfg.get("elevation", args.elevation, "GZ_VTOL_ELEVATION",
                   ORIGIN_ELEV_M, float)
    step = cfg.get("step_size", args.step_size, "GZ_VTOL_STEP_SIZE",
                   PHYSICS_STEP_SIZE, float)
    rtf = cfg.get("rtf", args.rtf, "GZ_VTOL_RTF", PHYSICS_RTF, float)
    ground = cfg.get("ground_size", args.ground_size, "GZ_VTOL_GROUND_SIZE",
                     GROUND_SIZE_M, float)
    fdm_addr = cfg.get("fdm_addr", args.fdm_addr, "GZ_VTOL_FDM_ADDR",
                       FDM_ADDR_DEFAULT, str)
    partition = cfg.get("gz_partition", args.gz_partition, "GZ_VTOL_PARTITION",
                        DEFAULT_GZ_PARTITION, str)
    model_path_raw = cfg.get("model_path", args.model_path, "GZ_VTOL_MODEL_PATH",
                             default_model_path, str)
    lift_min = cfg.get("lift_pwm_min", args.lift_pwm_min, "GZ_VTOL_LIFT_PWM_MIN",
                       LIFT_PWM_MIN_DEFAULT, int)
    lift_max = cfg.get("lift_pwm_max", args.lift_pwm_max, "GZ_VTOL_LIFT_PWM_MAX",
                       LIFT_PWM_MAX_DEFAULT, int)
    lift_ch_raw = cfg.get("lift_channels", args.lift_channels,
                          "GZ_VTOL_LIFT_CHANNELS", "", str)
    # "" = 不覆寫(走側寫 / 推論);"none" = 明確關掉。⚠️ 兩者必須分得開:
    #   把「沒設定」與「明確關閉」混為一談,就會出現「以為有檢查、其實沒有」。
    lift_override: Optional[Tuple[int, ...]] = None
    if lift_ch_raw.strip():
        if lift_ch_raw.strip().lower() == "none":
            lift_override = ()
            print("[warn] ⚠️ 已**明確關閉**升力馬達 PWM 刻度檢查(坑 7 的守衛)。"
                  "只有在確定這批機體沒有升力馬達時才該這樣。", file=sys.stderr)
        else:
            try:
                lift_override = tuple(sorted(
                    int(t) for t in lift_ch_raw.replace(" ", "").split(",") if t))
            except ValueError:
                raise SystemExit("✗ lift_channels=%r 解析失敗(要像 4,5,6,7)" % lift_ch_raw)
    min_instance = cfg.get("min_instance", args.min_instance,
                           "GZ_VTOL_MIN_INSTANCE", MIN_INSTANCE_DEFAULT, int)
    # ⚠️ 這裡原本寫死 `SettingRow("with_runway", ..., "cli", "-")` —— 就算沒下
    #    --no-runway 也回報「來源 = cli」。讀回表如果會說謊,它就不是讀回表了。
    #    順便補上環境變數覆寫(本專案要求每個可調常數都要能從 CLI 或 env 覆寫)。
    runway = cfg.get("with_runway", (False if not args.runway else None),
                     "GZ_VTOL_RUNWAY", True, _as_bool)
    cfg.dump()

    if step <= 0:
        raise SystemExit("✗ step_size 要 > 0")
    if lift_min >= lift_max:
        raise SystemExit("✗ lift_pwm_min(%d)要小於 lift_pwm_max(%d)" % (lift_min, lift_max))
    # ⚠️ 1 ms 以外的步長是**已知會壞**的(見 build_world 的註解)。允許覆寫,但要吵。
    if abs(step - PHYSICS_STEP_SIZE) > 1e-12:
        print("[warn] ⚠️ step_size=%s 不是 1 ms。swarm3.sdf 實測 2.5 ms 會讓 "
              "ArduCopter 速率環進滾轉極限環並一路沉到地面。確定要這樣?" % _f(step, 6),
              file=sys.stderr)

    model_path = _existing_dirs(model_path_raw)
    missing = [p for p in model_path if not p.is_dir()]
    for p in missing:
        # 不是致命的(路徑上本來就允許有還沒建立的目錄),但一定要說出來:
        # 靜靜跳過一個打錯的路徑,最後只會得到「找不到模型」這種不指向病灶的錯誤。
        print("[warn] ⚠️ 模型搜尋路徑不存在(略過):%s" % p, file=sys.stderr)

    # ── 機隊規格 ──────────────────────────────────────────────────────
    raw_specs = getattr(args, "fleet_specs", None) or []
    specs: List[dict] = []
    for kind, value in raw_specs:
        if kind == "add":
            specs.extend(_parse_add(value))
        else:
            specs.append(_parse_vehicle(value))
    if not specs:
        raise SystemExit(
            "✗ 沒有指定任何機體。例:--add quadplane_vtol:2 --add iris:2")

    fleet = plan_fleet(specs, instance_base, spacing, hdg,
                       args.spawn_z, args.spawn_roll, min_instance)

    # ── 解析模型 + 讀回逐通道刻度 ─────────────────────────────────────
    for veh in fleet:
        veh.model_sdf = find_model_sdf(veh.profile.model_dir, model_path)
        # ⚠️ sha1 先記下來:同一個模型檔在本次執行中會被解析 3~5 次
        #    (這裡一次、build_world 兩次 × 確定性自檢),而另一個 session 正在
        #    寫這個檔。中途被改掉 ⇒ ports.json 記的通道與世界檔裡嵌的不一致,
        #    且完全靜默。產完會再雜湊一次比對(見下面的 TOCTOU 讀回)。
        veh.model_sha1 = hashlib.sha1(veh.model_sdf.read_bytes()).hexdigest()
        _body, _plugin, channels, own_pose = inspect_model_body(veh.model_sdf)
        veh.channels = channels
        veh.model_own_pose = own_pose
        # 模型自己的 <pose> 才是作者算過起落架高度的那個值;側寫的 spawn_z 是
        # 「模型還沒生出來時的佔位值,無出處」。沒有更明確來源時以模型為準,
        # 並把來源印出來 —— 不要讓兩份常數各活各的。
        if own_pose is not None:
            if veh.z_source == "profile" and own_pose[2] != veh.z:
                print("[pose] %s:z 由側寫佔位值 %s 改成模型檔自己的 %s"
                      "(%s 的頂層 <pose>)"
                      % (veh.name, _f(veh.z, 3), _f(own_pose[2], 3),
                         veh.model_sdf.name), file=sys.stderr)
            if veh.z_source == "profile":
                veh.z, veh.z_source = own_pose[2], "model_own_pose"
            # roll/pitch 只有在模型明示 degrees="true" 時才敢採用(否則是弧度,
            # 當角度用會歪 57 倍)。inspect_model_body 對非 degrees 的情況回 nan。
            if veh.roll_source == "profile" and own_pose[3] == own_pose[3]:
                veh.roll_deg, veh.roll_source = own_pose[3], "model_own_pose"
            if veh.pitch_source == "profile" and own_pose[4] == own_pose[4]:
                veh.pitch_deg, veh.pitch_source = own_pose[4], "model_own_pose"
            rot = ("(旋轉分量沒有 degrees=\"true\" ⇒ 那是弧度,不採用,維持側寫值)"
                   if own_pose[3] != own_pose[3]
                   else " ".join(_f(v, 3) for v in own_pose[3:]))
            print("[pose] %s:模型自帶頂層 <pose> 平移 %s、旋轉 %s ⇒ 已丟棄改用世界檔的"
                  "(z 來源 %s / roll 來源 %s)"
                  % (veh.name, " ".join(_f(v, 3) for v in own_pose[:3]), rot,
                     veh.z_source, veh.roll_source), file=sys.stderr)
        elif veh.z_source == "profile":
            # ⚠️ 模型沒寫頂層 <pose> ⇒ 我們用的是側寫裡的**無出處佔位值**。
            #    這件事一定要說出來:靜靜用一個沒人算過的起落架高度,結果是穿地
            #    或自由落體,而 Gazebo 對這兩件事都不會報錯。
            print("[warn] ⚠️ %s:模型檔沒有頂層 <pose>,z 只能用側寫的佔位值 %s"
                  "(無出處)。請確認起落架高度,或用 --spawn-z / z= 明講。"
                  % (veh.name, _f(veh.z, 3)), file=sys.stderr)
        veh.lift_channels = lift_channels_for(veh, lift_override)
        check_lift_pwm(veh, veh.lift_channels, lift_min, lift_max)

    # ── 埠佔用實測 ────────────────────────────────────────────────────
    # ⚠️⚠️⚠️ plan_fleet 只保證「機隊**內部**的埠互不重複」。它擋不到「跟這台機器上
    #    別人的航次撞」—— 而 2026-09-09 `ss -lunp` 實測,另一個 session 的四機
    #    Gazebo 正佔著 UDP 9002/9012/9022/9032(pid 161034)、9005/9015/9025/9035,
    #    以及 5501/5511/5521/5531。撞到任何一個都是**完全靜默**的。
    #    這道檢查是單向的:ss 看到 ⇒ 一定撞,硬失敗;ss 沒看到 ⇒ 什麼都證明不了。
    busy, probe_note = probe_busy_udp_ports()
    port_probe = {"method": "ss -lun", "note": probe_note,
                  "ran": busy is not None, "conflicts": []}
    if busy is None:
        print("[warn] ⚠️ 埠佔用檢查**沒有執行**(%s)。這不是綠燈 —— "
              "起飛前請自行確認 fdm 埠沒被別人佔著。" % probe_note, file=sys.stderr)
    else:
        for veh in fleet:
            for key, port in udp_ports_for_instance(veh.instance).items():
                if port in busy:
                    port_probe["conflicts"].append(
                        {"vehicle": veh.name, "instance": veh.instance,
                         "port": port, "role": key, "seen_as": busy[port]})
        if port_probe["conflicts"]:
            lines = "\n".join(
                "    %s(instance %d)的 %s = UDP %d,已被 %s bind"
                % (c["vehicle"], c["instance"], c["role"], c["port"], c["seen_as"])
                for c in port_probe["conflicts"])
            msg = ("✗ 有 %d 個規劃中的 UDP 埠**現在就已經被 bind**:\n%s\n"
                   "  ⚠️ 撞埠不會報錯:SO_REUSEADDR 讓兩邊都 bind 成功,"
                   "後 bind 的收全部封包、先 bind 的收零\n"
                   "     (ArduPilotPlugin.cc:227 / SocketUDP.cc:72-75)。\n"
                   "  ⇒ 換一組 instance(--instance-base),"
                   "或確認那些行程已經結束後重跑。\n"
                   "  ⇒ 真的要硬上:--allow-busy-ports。"
                   % (len(port_probe["conflicts"]), lines))
            if not args.allow_busy_ports:
                raise SystemExit(msg)
            print("[warn] " + msg, file=sys.stderr)
        else:
            print("[port] ✔ %s,規劃中的 %d 個 UDP 埠都不在其中。"
                  "⚠️ 這**不代表**它們是空的:SO_REUSEADDR 讓佔用可以是隱形的,"
                  "真正的證明要靠啟動器送探測封包確認誰收到。"
                  % (probe_note, len(fleet) * 5), file=sys.stderr)

    # ── 對照表(先印,dry-run 也看得到) ───────────────────────────────
    print()
    print("[map] 第 k 架 = instance I = fdm 埠 P  ⚠️ 啟動器與 coordinator 要用同一份")
    print("[map] %-3s %-12s %-22s %-4s %-9s %-9s %-8s %s"
          % ("k", "name", "profile", "-I", "fdm_in", "sitl_in", "irlock", "channels(PWM)"))
    for veh in fleet:
        chans = " ".join("%d:%s/%s" % (c["channel"], c["servo_min"], c["servo_max"])
                         for c in veh.channels)
        print("[map] %-3d %-12s %-22s %-4d %-9d %-9d %-8d %s"
              % (veh.index, veh.name, veh.profile.key, veh.instance,
                 veh.ports["fdm_port_in"], veh.ports["sitl_sim_in"],
                 veh.ports["irlock_port"], chans))
    print()

    if args.dry_run:
        print("[gen] --dry-run:不寫檔、不驗證。")
        return 0

    # ── 產生 ──────────────────────────────────────────────────────────
    text = serialize(build_world(
        world_name, fleet, origin_lat=lat, origin_lon=lon, origin_elev=elev,
        hdg_deg=hdg, step_size=step, rtf=rtf, ground_size=ground,
        with_runway=runway, fdm_addr=fdm_addr))

    # 確定性自檢:同樣輸入再建一次,逐字比對。
    # ⚠️ 本專案踩過 str(callable) 帶記憶體位址造成「每次重跑 md5 都不同」的假 diff,
    #    而比 source_sha1 的守衛對那種退化完全瞎。這裡直接比產物本身。
    if args.determinism:
        again = serialize(build_world(
            world_name, fleet, origin_lat=lat, origin_lon=lon, origin_elev=elev,
            hdg_deg=hdg, step_size=step, rtf=rtf, ground_size=ground,
            with_runway=runway, fdm_addr=fdm_addr))
        if again != text:
            raise SystemExit(
                "✗ 確定性自檢失敗:同樣輸入建兩次結果不同。\n"
                "  不要產出這種檔 —— 它會製造永遠對不完的假 diff。")
        print("[gen] ✔ 確定性自檢通過(同輸入建兩次逐字相同)")

    out.parent.mkdir(parents=True, exist_ok=True)
    # 先寫暫存再驗再換上去:驗不過時**不留下**壞檔,免得下一個人撿到它去跑。
    tmp = out.with_name(out.name + ".generating.sdf")
    tmp.write_text(text, encoding="utf-8", newline="\n")
    # ⚠️⚠️ 順序很重要,原版是 gz 驗 → os.replace → 讀回 → 寫 ports.json。
    #    那個順序有一個靜默的分歧窗:讀回(或 md5)一旦丟例外,**新的世界檔已經
    #    在磁碟上了**,而旁邊的 ports.json 還是上一輪的舊值 ⇒ 啟動器讀到的埠
    #    對不上世界檔裡的埠,而這正是【坑 1】的形狀,且沒有任何東西會報錯。
    #    ⇒ 改成:所有驗證(gz sdf -k、逐架讀回、模型檔 TOCTOU)都在**暫存檔**上做完,
    #      全過了才一次換上去,而且換之前先把舊的 ports.json 移走。
    try:
        gz_sdf_check(tmp, model_path)
        print("[gen] ✔ gz sdf -k 驗證通過(SDF_PATH=%s)"
              % ":".join(str(p) for p in model_path))
        readback_world(tmp, fleet)
        print("[gen] ✔ 讀回通過:%d 架的 model 名與 fdm 埠與規劃一致" % len(fleet))
        # TOCTOU:模型檔在本次執行中被解析多次,而另一個 session 正在寫它。
        # 重新雜湊比對,不一致就整個作廢重跑 —— 半新半舊的世界檔比沒有還糟。
        for veh in fleet:
            assert veh.model_sdf is not None
            now = hashlib.sha1(veh.model_sdf.read_bytes()).hexdigest()
            if now != veh.model_sha1:
                raise SystemExit(
                    "✗ %s 的模型檔在本次產生過程中被改掉了(sha1 %s → %s):\n"
                    "    %s\n"
                    "  ⇒ ports.json 記的通道可能已經與世界檔裡嵌的不一致。"
                    "請等對方寫完後重跑,不要用這一份。"
                    % (veh.name, veh.model_sha1[:12], now[:12], veh.model_sdf))
        print("[gen] ✔ 模型檔 sha1 未變(%d 架)" % len(fleet))
    except SystemExit:
        tmp.unlink(missing_ok=True)
        raise

    world_md5 = hashlib.md5(tmp.read_bytes()).hexdigest()
    table = build_ports_table(world_name, out, fleet, cfg, origin_lat=lat,
                              origin_lon=lon, origin_elev=elev, hdg_deg=hdg,
                              gz_partition=partition, model_path=model_path,
                              fdm_addr=fdm_addr, world_md5=world_md5,
                              port_probe=port_probe, min_instance=min_instance)
    ports_path = out.with_name(out.name + ".ports.json")
    ports_tmp = ports_path.with_name(ports_path.name + ".generating.json")
    ports_tmp.write_text(
        json.dumps(table, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8", newline="\n")
    # 舊的對照表先拿掉,再一起換上去:任何一步掛掉都不會留下「新世界檔 + 舊對照表」。
    ports_path.unlink(missing_ok=True)
    os.replace(str(tmp), str(out))
    os.replace(str(ports_tmp), str(ports_path))
    print("[gen] 世界檔 → %s  (md5 %s)" % (out, world_md5))
    print("[gen] 對照表 → %s" % ports_path)

    # ── 啟動提示(本檔不啟動任何東西) ────────────────────────────────
    print()
    print("[gen] ⚠️ 本檔只寫檔。起 Gazebo 前:")
    print("[gen]   1) export GZ_PARTITION=%s   ← 與別人的 Gazebo 隔離,必要" % partition)
    print("[gen]   2) export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build")
    print("[gen]      export GZ_SIM_RESOURCE_PATH=%s"
          % ":".join(str(p) for p in model_path))
    print("[gen]      ⚠️ 這兩個只寫在 ~/.bashrc,非互動 shell 讀不到,而 gz 的錯誤訊息"
          "不會指向這個原因。")
    print("[gen]   3) SITL 逐機:arduplane --model JSON -I<instance> "
          "--home %.6f,%.6f,%s,%s" % (lat, lon, _f(elev, 1), _f(hdg, 1)))
    print("[gen]      ⚠️ 用現有的 build/sitl/bin/arduplane,跑 sim_vehicle.py 要加 "
          "--no-rebuild(不要重建 ~/ardupilot,那棵樹有別人未提交的改動)。")
    print("[gen]      ⚠️ 直接叫 arduplane 起 SITL 一定要 setsid,否則腳本一結束"
          "子程序就被收掉,而症狀會長得像「Gazebo 收不到 FDM」。")
    print("[gen]      ⚠️ SERIAL0 預設是 tcp:<port>:wait,會阻塞在 accept()"
          "(UARTDriver.cpp:415-427):沒有 GCS 接上就完全不跑 FDM 迴圈,"
          "表徵與「收不到 FDM」一模一樣。")
    print("[gen]      ⇒ 要嘛每架都掛連線,要嘛用不等待的 serial0"
          "(字面旗標寫在 ports.json 的 sitl_serial0_note,"
          "因為 XML 註解不能含連續兩個減號)。")
    print("[gen]      ⚠️ instance 決定埠,埠寫錯是靜默的 —— 用上面對照表的值,"
          "不要自己再算一次。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
