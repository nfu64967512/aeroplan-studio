#!/usr/bin/env bash
# gz_vtol_up.sh — VTOL 群機:Gazebo(Harmonic / gz-sim 8)+ N 台 SITL,**同一個 world 可混機種**。
#
# 定位:這是**隔離啟動器**。它只做四件事,每一件都要留下可讀回的證據:
#   §5 用 gz_vtol_world.py 產世界檔,並**逐值讀回**產物(不是相信它有跑)
#   §6 用 gz_port_probe.py 確認埠位真的乾淨(送探測封包,不是看 ss)
#   §7-8 起 gz + 逐架 SITL(逐機挑對的 binary 與參數底座)
#   §9 **逐機**證明每一架都活著(sim_time 前進不算數)
#
# 停止:bash gz_vtol_down.sh(只收我們自己起的,靠 GZ_VTOL_RUN_ID 指認)
#
# ═══════════════════════════════════════════════════════════════════════════
# 為什麼這支腳本長這樣(踩過的坑,每一條都附出處)
# ═══════════════════════════════════════════════════════════════════════════
#
# ⚠️⚠️⚠️ 【埠】SO_REUSEADDR 會讓重複的 UDP bind **成功**。
#   ArduPilotPlugin 的 socket 是 `SocketUDP(true, true)`
#   (ardupilot_gazebo/src/ArduPilotPlugin.cc:227 → SocketUDP.cc:71-75 設 SO_REUSEADDR),
#   本機 2026-09-09 實測:兩個 SO_REUSEADDR socket 都 bind 127.0.0.1:19003 成功,
#   **後 bind 的收到全部封包、先 bind 的收到零**,無錯誤、無 log。
#   plugin 的 `gzerr << "failed to bind"`(ArduPilotPlugin.cc:1289-1293)永遠不會印。
#   ⇒ `ss -lunp` 看到「埠都 bind 了」**證明不了任何事**。
#   ⇒ 判決一律交給 gz_port_probe.py 的獨占 bind(不設 SO_REUSEADDR)。
#
# ⚠️⚠️ 【判準】lock_step 的失效語意(ArduPilotPlugin.cc:1204-1213 / 1523-1548):
#   * 已上線過的機體斷線 ⇒ arduPilotOnline 不會被設回 false ⇒ PreUpdate 卡在
#     `while (!ReceiveServoPacket() && arduPilotOnline)` 每輪睡 10 ms。
#     gz-sim 的 PreUpdate 在**單一更新執行緒** ⇒ **一架死掉會凍結整個 world**,
#     而且因為是 sleep 不是 busy-spin,**沒有 CPU 尖峰**(用 CPU 判別會失效)。
#   * 從沒收到第一個封包的機體 ⇒ arduPilotOnline 仍 false ⇒ 迴圈立刻退出 ⇒
#     **world 照跑,那架靜靜死掉**。
#   ⇒ 「sim_time 有前進」是**必要但不充分**。本腳本的閘門是逐機的(§9),
#     sim_time 只印出來當參考資訊,不參與判決。
#
# ⚠️⚠️ 【埠與 instance 綁死】ArduPilot 的埠是從 instance 號推的,不是各算各的:
#   SITL_cmdline.cpp:258-260  SIM_OUT_PORT=9002 / SIM_IN_PORT=9003 / IRLOCK_PORT=9005
#   SITL_cmdline.cpp:435-452  `-I` 之後,**凡是還等於預設值的**一律 += instance*10
#   ⇒ SDF 的 <fdm_port_in> 必須 == 該架的 9002+10*instance;
#     錯位的症狀是 "No JSON sensor message",與撞名一模一樣。
#   ⇒ 本腳本**不自己編一套埠**:以 gz_vtol_world.py 產出的 `<world>.ports.json`
#     為主,再用 gz_port_probe.py 的算術獨立推一次來**對帳**。
#     ⚠️ 兩份實作算出同一個數字**不等於**正確(兩邊可以一起錯);
#       這道對帳抓的是**分歧**,正確性的錨是上面那兩行 ArduPilot 原始碼。
#
# ⚠️⚠️ 【SIM_IN_PORT 不要檢查】`JSON::set_interface_ports()`
#   (ardupilot/libraries/SITL/SIM_JSON.cpp:88-99)只做 set_blocking + reuseaddress,
#   **從頭到尾沒有 bind**。9003+10i 在 `ss` 上根本不存在,列進必檢清單只會製造假紅燈。
#
# ⚠️⚠️ 【TCP 基準埠】任務書寫「-I10 ⇒ SITL TCP 6360+」,但韌體算術是
#   5760+10*10 = **5860**,不是 6360;而 5860+10i 正是 sar_swarm 的 MAVROS 埠段
#   (swarm_sitl_up.sh 的埠位表)。⇒ 本腳本**顯式**傳 `--base-port`,把 SITL 的
#   TCP 段挪到 6360 起,和固定翼線完全不重疊(實測 6300-6499 目前全空)。
#   ⚠️ `--base-port 5760` 不可用:SITL_cmdline.cpp:435 的守衛是 `if (_base_port ==
#     BASE_PORT)`,顯式給 5760 之後 `-I` 還是會加 instance*10,我們就再也不知道
#     它 listen 在哪。gz_port_probe.py 會擋掉這個值。
#
# ⚠️ 【SERIAL0 的 accept 閘門】SITL 的 serial0 預設是 `tcp:0:wait`
#   (AP_HAL_SITL/SITL_State.h:29-30),`wait_for_connection` 會**阻塞在 accept()**
#   (UARTDriver.cpp:415-427),沒有 GCS 接上就完全不跑 FDM 迴圈,
#   表徵和「收不到 FDM」一模一樣。
#   ⇒ **本腳本預設選 `--serial0 tcp:0`(不帶 :wait)**,理由:
#     ① §9 的逐機驗活閘門必須能在 coordinator 接上**之前**跑完,
#        否則「起不來」和「還沒人連」永遠分不開;
#     ② 要求每架都先掛一個 client,等於把第 1 層綁死在第 2 層上,
#        而本專案已經有「一個 tcpin 埠只餵第一個 client」的慘痛前例
#        —— 少一個共用點就少一類靜默失敗。
#     `tcp:0` 的 0 是 base_port + 0(UARTDriver.cpp:368-372,N ≤ 1000 才是偏移),
#     所以 listen 埠不變,只是不等人。
#   ⇒ 要回到舊行為:`SERIAL0_MODE=wait bash gz_vtol_up.sh`。
#   ⇒ 這個選擇本身也**讀回驗證**:nowait 模式下每架的 log 都不可以出現
#     "Waiting for connection ...."(§9 的 G3)。
#
# ⚠️ 【環境變數】GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH 只寫在 .bashrc
#   (本機實測連 .bashrc 裡都**沒有**),非互動 shell 一定是空的,而 gz 的錯誤訊息
#   不會指向這個原因。⇒ 這裡明確 export,並在 export 後**讀回檢查**。
#   ⚠️ `gz sdf -k` 驗有 <include> 的世界檔時,關鍵變數是 **SDF_PATH**,不是
#     GZ_SIM_RESOURCE_PATH。本機實測:
#       有 SDF_PATH → "Valid."(rc=0);沒有 → "Error Code 14: Unable to find uri[model://...]"
#     (順帶更正:scripts/wsl/gz_up.sh 裡「不要用 gz sdf -k,它解析不了 model://」
#      那段註解只對了一半 —— 缺的是 SDF_PATH。那個檔不歸本任務改,記在這裡備查。)
#   ⚠️ model://ground_plane 本機沒有(會去 Fuel 線上抓)⇒ 離線 Error Code 14。
#     地面要內嵌 —— 那是 gz_vtol_world.py 的事,這裡只負責在 §5 驗它有沒有做到。
#
# ⚠️ 【setsid】直接叫 arduplane/arducopter 起 SITL 一定要 setsid,否則它們留在
#   本終端的 process group / session 裡,Ctrl+C(SIGINT 送整個前景 group)或
#   關終端(SIGHUP)會順手收掉飛機。
#   (查證:ArduPilot SITL 樹裡 grep 不到 PR_SET_PDEATHSIG,所以 setsid 就夠了,
#    不需要雙重 fork。)
#
# ⚠️ 【GZ_PARTITION】gz-transport 的分割區(libgz-transport13.so.13 內確認有這個
#   字串;IGN_PARTITION 已無)。設了之後我們的 topic/service 和別人的 gz 完全隔開。
#   ⚠️⚠️ 但它**只隔 transport**:FDM 的 UDP 埠是普通 socket,**不受 partition 保護**。
#     所以 partition 不能取代埠位檢查,兩個都要。
#
# ⚠️ 【世界原點】--home 必須與 world 的 spherical_coordinates 一致,否則座標基準
#   差開而且完全沒有告警。§5 會逐值比對。
#   ⚠️⚠️ 桃園世界檔有個未解問題:地面在世界 z=0,而原點 elevation 寫 32 m
#     ⇒ Gazebo 的「海平面」其實是 32 m AMSL。這次照 32 做,每趟都印出來提醒。
#
# ⚠️ 【參數底座】QuadPlane 用 ~/ardupilot/Tools/autotest/default_params/quadplane.parm
#   (上游調過的完整基準),**不要**從 mission/sitl_launcher.py 的手寫 vtol_default.parm
#   起頭。⚠️ plane.parm **不存在**,別寫進指令。
#   ⚠️⚠️ 值層級守衛(§4):合併後的 defaults 裡若出現 `ARMING_REQUIRE 0` 直接中止 ——
#     它會讓 QuadPlane 的四軸馬達停在 PWM 1000(本專案實測過的災難)。
#   ⚠️⚠️ Q_M_PWM_MIN/MAX 必須是 1000/2000,而且要和世界檔升力通道的
#     servo_min/servo_max **對得上**(§5 逐值比對)。照抄 iris 的 1100/1900 會讓
#     ArduPilotPlugin.cc:1673-1675 的 raw_cmd=(pwm-1100)/800 把上下各 12.5% 夾掉,
#     四顆一起損失 25% 行程,而且完全沒有訊息。
#     ⚠️ Q_M_PWM_TYPE 預設 NORMAL ⇒ SERVOn_MIN/MAX 對馬達輸出**不生效**,
#       生效的是 Q_M_PWM_MIN/MAX。
#
# ⚠️⚠️ 【異質機隊】同一個 world 混機種時,**每一架的 SITL binary 與參數底座都不一樣**:
#   iris = arducopter + gazebo-iris.parm,zephyr/quadplane = arduplane + 各自的 parm。
#   全部用 arduplane 起 iris 會「起得來、飛不動」而且不會有明顯錯誤。
#   ⚠️ 而且串流預設也不一樣:GCS_MAVLink_Parameters.cpp:49-59 ArduPlane 預設
#     POSITION=1 / EXTRA1=1 Hz,:60-71 ArduCopter 預設**全部 0**
#     ⇒ 同一份 coordinator 在 plane 上看得到 1 Hz、在 copter 上全黑。
#     本層**刻意不代打**(見 §4 的 overlay 註解)。
#
# ⚠️ 【不動別人的樹】不重建 arduplane/arducopter(~/ardupilot 有別的 session 未提交的
#   改動,waf 開著 autoconfigure)。JSON 後端自 HEAD 以來未變、SIM_JSON.cpp.0.o 已封存,
#   直接用現有的 build/sitl/bin/*。本腳本**不呼叫 waf、不呼叫 sim_vehicle.py**。
#
# ═══════════════════════════════════════════════════════════════════════════
# 驗證紀錄(2026-09-09,DRY_RUN 全程,沒有起過任何 gz/SITL)
# ═══════════════════════════════════════════════════════════════════════════
# §5 的「世界檔逐值讀回」做過突變實驗,**15/15 臂符合預期**:
#   M0  未突變的基準                       → 綠(證明不是卡死紅燈)
#   M1  世界檔某個 fdm_port_in 被改掉        → 紅
#   M2  世界檔的 fdm_port_in 重複            → 紅
#   M3  某架 lock_step 改成 0               → 紅
#   M4  原點緯度平移 1°                      → 紅
#   M4b 原點 elevation 改成 0               → 紅
#   M4c 整段 spherical_coordinates 拿掉      → 紅
#   M5  ports.json 的 world_md5 被弄髒(殘留的舊對照表)→ 紅
#   M6  ports.json 不存在                    → 紅
#   M7  ports.json 的 fdm 埠不合 9002+10*I   → 紅
#   M8  ports.json 的 instance 重複          → 紅
#   M9  世界檔 include 了 model://ground_plane(離線抓不到)→ 紅
#   M10 升力通道刻度用 iris 的 1100/1900     → 紅
#   M11 QuadPlane 但 lift_channels_checked 是空的 → 紅
#   M12 FLEET 順序與 ports.json 不一致        → 紅
#   ⚠️ 第一版的 M4 用了對不上格式的 sed(25.0777 vs 25.077700)⇒ **突變根本沒套用**,
#     那一臂變成「什麼都沒改卻期望報紅」的空測試,而空測試永遠是綠燈。
#     突變腳本因此加了「檔案 md5 必須真的改變」的前置斷言。
#     教訓:每一條測試都要先斷言「這個設定真的能讓 X 發生」。
#
# gz_port_probe.py 的 --self-test 是 6/6;另外拿**真的**被佔的埠段對抗過:
#   `python3 gz_port_probe.py --i0 0 -n 1 --tcp-base 5761`
#   → 抓到別的 session 的 gz(ruby)以 SO_REUSEADDR 佔著 127.0.0.1:9002 並指名 pid。
#
# ═══════════════════════════════════════════════════════════════════════════
# 用法
# ═══════════════════════════════════════════════════════════════════════════
#   bash gz_vtol_up.sh                                   # 4 架 quadplane,I=10..13,headless
#   N=2 bash gz_vtol_up.sh
#   FLEET="quadplane_vtol:2 iris:1 zephyr:1" bash gz_vtol_up.sh   # ← 混機種
#   HEADLESS=0 bash gz_vtol_up.sh                        # 要 WSLg
#   I0=20 TCP_BASE=6460 bash gz_vtol_up.sh               # 挪開埠段
#   WORLD=/path/x.sdf bash gz_vtol_up.sh                 # 用現成世界檔(仍會做 §5 讀回)
#   DRY_RUN=1 bash gz_vtol_up.sh                         # 只做檢查與產檔,不起任何行程
#   PROBE_SELFTEST=0 bash gz_vtol_up.sh                  # 跳過驗證器自檢(不建議)
#
# 離開碼:0 全綠 / 1 前置檢查擋下 / 2 用法或環境錯 / 3 驗證器自檢失敗 / 4 逐機驗活失敗
# ═══════════════════════════════════════════════════════════════════════════
set -uo pipefail

SR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="$SR/gz_port_probe.py"
WORLD_GEN="${WORLD_GEN:-$SR/gz_vtol_world.py}"

# ── 可覆寫常數(全部會被讀回並寫進 state 檔)──────────────────────────────
N="${N:-4}"
FLEET="${FLEET:-}"                   # 空 = quadplane_vtol:$N;給了就以它為準(N 由它推)
I0="${I0:-10}"                       # instance 0..3 被另一個 session 的四機 Gazebo 佔著
TCP_BASE="${TCP_BASE:-6360}"         # 第一架的 SITL TCP 基準(見上面「TCP 基準埠」)
GZ_PARTITION="${GZ_PARTITION:-vtolswarm}"
HOME_LL="${HOME_LL:-25.0777,121.2325,32,52}"   # 桃園跑道 05;必須與世界原點一致
AG="${AG:-$HOME/ardupilot_gazebo}"
AP="${AP:-$HOME/ardupilot/build/sitl/bin/arduplane}"
AC="${AC:-$HOME/ardupilot/build/sitl/bin/arducopter}"
APD="${APD:-$HOME/ardupilot/Tools/autotest/default_params}"
QP_PARM="${QP_PARM:-$APD/quadplane.parm}"
ZEPHYR_PARM="${ZEPHYR_PARM:-$APD/gazebo-zephyr.parm}"
IRIS_PARM="${IRIS_PARM:-$APD/gazebo-iris.parm}"
EXTRA_PARM="${EXTRA_PARM:-}"         # 逗號分隔,疊在每一架的最後
RUN="${RUN:-$HOME/vtol_ws/gz_run}"
# ⚠️ 三個 overlay 的**路徑**要在這裡就定好。內容在 §4 才寫,但 base_parm_for()
#   在 §1 就會被呼叫 ⇒ 放在 §4 定義的話 `set -u` 會直接 unbound variable,
#   而錯誤訊息是「profile iris 需要參數底座 ,但它不存在」—— 指向錯的地方。
OVERLAY_COMMON="$RUN/vtol_common.parm"
OVERLAY_QP="$RUN/vtol_quadplane.parm"
OVERLAY_COPTER="$RUN/vtol_copter.parm"
WORLD="${WORLD:-}"                   # 空 = 由 WORLD_GEN 產生
WORLD_NAME_REQ="${WORLD_NAME:-vtolswarm}"
SPACING="${SPACING:-25}"
HEADLESS="${HEADLESS:-1}"
SPEEDUP="${SPEEDUP:-1}"
WIPE="${WIPE:-1}"                    # -w:每趟清 eeprom,保證吃到的是 --defaults 而非殘留
SERIAL0_MODE="${SERIAL0_MODE:-nowait}"   # nowait | wait
PROBE_SELFTEST="${PROBE_SELFTEST:-1}"
GZ_WAIT_S="${GZ_WAIT_S:-45}"
SITL_WAIT_S="${SITL_WAIT_S:-60}"
LIVE_WINDOW_S="${LIVE_WINDOW_S:-3}"
DRY_RUN="${DRY_RUN:-0}"

# 這一趟的唯一標記。gz_vtol_down.sh 靠它從 /proc/<pid>/environ 精準指認我們的行程,
# **不用 pkill -f**。本專案有 pkill 自殺與誤殺的前科,而且這台機器上就有別人的航次。
GZ_VTOL_RUN_ID="${GZ_VTOL_RUN_ID:-vtol-$(date +%Y%m%d-%H%M%S)-$$}"
export GZ_VTOL_RUN_ID GZ_PARTITION

STATE="$RUN/vtol_swarm.env"
say()  { printf '[vtol] %s\n' "$*"; }
warn() { printf '[vtol] ⚠️  %s\n' "$*" >&2; }
# ⚠️ die 的第二個位置參數是**離開碼**,而第一版用 "$*" 把所有引數串起來
#    ⇒ 每一次 die "…" 1 都會在訊息尾巴多印一個裸的 " 1",看起來像訊息本身。
die()  { printf '[vtol] ✗ %s\n' "$1" >&2; exit "${2:-2}"; }

# ═══════════════════════════════════════════════════════════════════════════
# §1 前提檔案
# ═══════════════════════════════════════════════════════════════════════════
say "══ §1 前提檔案 ══"
[ -x "$AP" ]                             || die "找不到 arduplane:$AP(不要重建,問另一個 session)"
[ -f "$AG/build/libArduPilotPlugin.so" ] || die "找不到 $AG/build/libArduPilotPlugin.so"
[ -f "$QP_PARM" ]                        || die "找不到 QuadPlane 參數底座:$QP_PARM"
[ -f "$PROBE" ]                          || die "找不到埠位檢查:$PROBE"
command -v gz      >/dev/null 2>&1 || die "PATH 上沒有 gz"
command -v setsid  >/dev/null 2>&1 || die "PATH 上沒有 setsid"
command -v python3 >/dev/null 2>&1 || die "PATH 上沒有 python3"
say "arduplane : $AP  ($(date -r "$AP" '+%F %T'))"
say "plugin    : $AG/build/libArduPilotPlugin.so"
say "gz        : $(gz sim --versions 2>/dev/null | head -1)"
mkdir -p "$RUN" || die "建不了工作目錄 $RUN"

# ── 機隊展開 ──────────────────────────────────────────────────────────────
[ -n "$FLEET" ] || FLEET="quadplane_vtol:$N"
PROFILES=()
for item in $FLEET; do
  prof="${item%%:*}"; cnt="${item##*:}"
  [ "$prof" = "$item" ] && cnt=1
  case "$cnt" in ''|*[!0-9]*) die "FLEET 項 '$item' 的數量不是整數" ;; esac
  for _ in $(seq 1 "$cnt"); do PROFILES+=("$prof"); done
done
[ "${#PROFILES[@]}" -ge 1 ] || die "FLEET 展開後是空的:'$FLEET'"
if [ -n "${N:-}" ] && [ "${#PROFILES[@]}" -ne "$N" ] && [ -n "${FLEET_EXPLICIT:-1}" ]; then
  # FLEET 是唯一真相;N 只是沒給 FLEET 時的捷徑。兩者不合時大聲改寫,不要靜靜用其中一個。
  [ "$FLEET" = "quadplane_vtol:$N" ] || \
    warn "N=$N 與 FLEET='$FLEET'(共 ${#PROFILES[@]} 架)不一致 —— **以 FLEET 為準**"
fi
N="${#PROFILES[@]}"
say "機隊:$FLEET  ⇒ N=$N  (${PROFILES[*]})"

# 逐架挑 SITL binary 與參數底座。混機種時這一步錯了,飛機會「起得來、飛不動」。
sitl_binary_for() {
  case "$1" in
    iris|iris_*|*copter*) printf '%s' "$AC" ;;
    *)                    printf '%s' "$AP" ;;   # quadplane_vtol / zephyr / 未知都當 plane
  esac
}
base_parm_for() {
  case "$1" in
    iris|iris_*|*copter*) printf '%s' "$IRIS_PARM" ;;
    zephyr|zephyr_*)      printf '%s' "$ZEPHYR_PARM" ;;
    *)                    printf '%s' "$QP_PARM" ;;
  esac
}
# ⚠️⚠️ 上面兩個 case 的 `*)` 是**靜默的**萬用分支:任何認不得的 profile 名字
#   都會拿到 arduplane + quadplane.parm。混機種時這一步錯了,飛機會
#   「起得來、飛不動」而且沒有任何錯誤訊息 —— 正是本檔要防的那類失效。
#   fallback 本身要留(VTOL 模型由另一個 session 產出,目錄名還可能變;
#   gz_vtol_world.py:388-406 的 resolve_profile 也是同樣的設計),
#   但**必須大聲**,而且要能從命令列覆寫:
#     PROFILE_BIN_<profile>=/path/to/binary
#     PROFILE_PARM_<profile>=/path/to.parm
#   (profile 名字裡的非英數字元一律換成 _)
known_profile() {
  case "$1" in
    iris|iris_*|*copter*|zephyr|zephyr_*|quadplane|quadplane_*) return 0 ;;
    *) return 1 ;;
  esac
}
profile_key() { printf '%s' "$1" | tr -c 'A-Za-z0-9' '_'; }
for p in "${PROFILES[@]}"; do
  pk="$(profile_key "$p")"
  eval "ovb=\${PROFILE_BIN_$pk:-}"
  eval "ovf=\${PROFILE_PARM_$pk:-}"
  b="${ovb:-$(sitl_binary_for "$p")}"; f="${ovf:-$(base_parm_for "$p")}"
  if ! known_profile "$p"; then
    warn "profile '$p' 不在已知清單裡 ⇒ 落到萬用分支:$(basename "$b") + $(basename "$f")。
     它若其實是多旋翼,用 arduplane 起會**起得來但飛不動**而且沒有錯誤訊息。
     要指定就設 PROFILE_BIN_$pk=... / PROFILE_PARM_$pk=..."
  fi
  [ -n "$ovb" ] && say "  profile '$p' 的 binary 由 PROFILE_BIN_$pk 覆寫 → $b"
  [ -n "$ovf" ] && say "  profile '$p' 的參數底座由 PROFILE_PARM_$pk 覆寫 → $f"
  [ -x "$b" ] || die "profile '$p' 需要 $b,但它不存在/不可執行"
  [ -f "$f" ] || die "profile '$p' 需要參數底座 $f,但它不存在"
done

# ═══════════════════════════════════════════════════════════════════════════
# §2 環境變數:明確 export + 讀回
# ═══════════════════════════════════════════════════════════════════════════
say "══ §2 環境變數 ══"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
# ⚠️ 我們自己的機體(quadplane_vtol)住在**本 repo** 的 sitl/gazebo/models/,
#   不在 ardupilot_gazebo 底下。四個基件是各自獨立寫的,這一行是它們的接縫:
#   漏掉它的症狀是世界產生器報「找不到模型 quadplane_vtol 的 model.sdf」。
#   放在最前面,讓我們的模型優先於同名的上游模型(目前沒有同名,但不要靠運氣)。
REPO_MODELS="${REPO_MODELS:-$(cd "$SR/../.." && pwd)/sitl/gazebo/models}"
[ -d "$REPO_MODELS" ] || warn "本 repo 的模型目錄不存在:$REPO_MODELS(混機種只剩上游模型可用)"
MODEL_PATH="$REPO_MODELS:$AG/models:$AG/worlds:$RUN/models"
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# ⚠️ SDF_PATH 才是 `gz sdf -k` 解析 model:// 用的那一個(實測見檔頭)。
export SDF_PATH="$MODEL_PATH${SDF_PATH:+:$SDF_PATH}"

# 讀回:不是「有設」就算,是「設進去的東西真的在」。
case ":$GZ_SIM_SYSTEM_PLUGIN_PATH:" in
  *":$AG/build:"*) ;;
  *) die "GZ_SIM_SYSTEM_PLUGIN_PATH 讀回失敗:$GZ_SIM_SYSTEM_PLUGIN_PATH" ;;
esac
for d in "$AG/models" "$AG/worlds"; do [ -d "$d" ] || die "resource path 裡的 $d 不存在"; done
say "GZ_PARTITION              = $GZ_PARTITION   (只隔 transport,**不保護 UDP 埠**)"
say "GZ_VTOL_RUN_ID            = $GZ_VTOL_RUN_ID"
say "GZ_SIM_SYSTEM_PLUGIN_PATH = $GZ_SIM_SYSTEM_PLUGIN_PATH"
say "GZ_SIM_RESOURCE_PATH      = $GZ_SIM_RESOURCE_PATH"
say "SDF_PATH                  = $SDF_PATH"

# ═══════════════════════════════════════════════════════════════════════════
# §3 驗證器自檢:先證明「埠位檢查本身會響」
#    本專案的教訓:驗證器壞掉時表徵一律是綠燈 ⇒ 每趟都先讓它在明知被佔的埠上報紅。
# ═══════════════════════════════════════════════════════════════════════════
if [ "$PROBE_SELFTEST" = "1" ]; then
  say "══ §3 埠位檢查自檢 ══"
  python3 "$PROBE" --self-test || die "埠位檢查自檢失敗 —— 它的綠燈不可信,擋下這趟" 3
else
  warn "§3 已被 PROBE_SELFTEST=0 跳過 —— 之後的『埠乾淨』是**未經驗證的綠燈**"
fi

# ═══════════════════════════════════════════════════════════════════════════
# §4 參數:合併 + 值層級守衛
# ═══════════════════════════════════════════════════════════════════════════
say "══ §4 參數 ══"
# 路徑已在上面定義;這裡只寫內容。
cat > "$OVERLAY_COMMON" <<'PARM_EOF'
# 由 gz_vtol_up.sh 產生 —— 所有機種共用的差異項。目前刻意是空的,只留說明。
#
# ⚠️⚠️ **刻意不設 MAV0_/MAV1_/MAV2_* 串流速率。**
#   異質機隊的靜默陷阱:GCS_MAVLink_Parameters.cpp:49-59 ArduPlane 預設
#   POSITION=1 / EXTRA1=1 Hz,:60-71 ArduCopter 預設**全部 0**。
#   在這裡把速率填滿,會讓 plane 和 copter 看起來一樣正常,於是 coordinator
#   「連上後必須無條件請求串流並斷言真的收到」這條義務就永遠不會被測到 ——
#   直到某天混進一台 copter 才整片變黑。
#   ⇒ 這一層維持原廠預設,把責任留在 coordinator 那一側**並讓它可被檢驗**。
#   ⚠️ 順帶:串流速率參數**已改名**成 MAV0_/MAV1_/MAV2_*;舊的 SR0_/SR1_/SR2_*
#     會被韌體**靜默略過**(ArduPilot 4.7/4.8 對 --defaults 的未知參數名不報錯,
#     內建 SITL 就這樣死掉過 32 個參數)。
PARM_EOF
cat > "$OVERLAY_QP" <<'PARM_EOF'
# 由 gz_vtol_up.sh 產生 —— 只給 QuadPlane。底座是上游的 quadplane.parm
# (Q_ENABLE 1 / Q_M_PWM_MIN,MAX 1000,2000 / Q_M_THST_EXPO 0.65 / 整套 Q_A_RAT_*、
#  Q_A_ANG_* / SCHED_LOOP_RATE 300 / Q_ASSIST_SPEED 18 / ARSPD_USE 1 / INS 假校正 /
#  COMPASS 偏置),不要在這裡重寫底座已經有的東西。

# 機架。寫出來只為了讓它**可見**並可進契約表 —— 1/1 = Quad / X,對上 iris 那四顆
# rotor 的 AP quad-X 佈局(M1 CCW 右前 / M2 CCW 左後 / M3 CW 左前 / M4 CW 右後)。
# 預設本來就是這兩個值,行為逐位元不變。
Q_FRAME_CLASS 1
Q_FRAME_TYPE  1
PARM_EOF

cat > "$OVERLAY_COPTER" <<'PARM_EOF'
# 由 gz_vtol_up.sh 產生 —— 只給多旋翼(iris)。底座是上游的 gazebo-iris.parm。
#
# ⚠️⚠️⚠️ **馬達 PWM 行程必須對齊模型的 servo 刻度**,而上游兩邊是不一致的:
#   ~/ardupilot_gazebo/models/iris/model.sdf 的四個 <control> 都是
#     <servo_min>1100</servo_min> / <servo_max>1900</servo_max>
#   而 ArduCopter 的 MOT_PWM_MIN/MAX 出廠是 1000/2000,
#   上游的 gazebo-iris.parm **沒有**處理這件事(整份只有 FRAME_CLASS/TYPE 與 IRLOCK/RNGFND)。
#   後果:ArduPilotPlugin.cc:1673-1675 算的是 raw_cmd = (pwm - servo_min)/(servo_max - servo_min),
#   飛控送 1000 時 raw_cmd = -0.125 被夾成 0、送 2000 時 1.125 被夾成 1
#   ⇒ 上下各 12.5%、四顆一起損失 25% 行程。
#   ⚠️ 這**不會報任何錯**,只表現成「推力怪怪的 / 爬升比預期慢」,
#     而那種症狀會把人導去調 PID,不會導去看 PWM 刻度。
#   2026-09-10 由 gz_vtol_coordinator.py 的逐機參數契約在實機上抓到。
#   ⇒ 改飛控這一側(1100/1900),不改上游模型:模型是別人的檔,而且我們自己的
#     quadplane_vtol 走的是另一組刻度(1000/2000,與 Q_M_PWM_MIN/MAX 對齊)。
MOT_PWM_MIN 1100
MOT_PWM_MAX 1900
PARM_EOF

# --defaults 吃逗號分隔的清單:AP_Param.cpp:2406-2427 用 strtok_r(",") 逐檔載入,
# 後面的檔覆蓋前面的。少一個檔 load_defaults_file() 直接 return false,
# 而症狀只會是「參數怎麼跟我想的不一樣」⇒ 這裡逐檔確認存在。
# ⚠️⚠️⚠️ EXTRA_PARM **必須**在這裡疊,不可以留到 §8 才接。
#   第一版是 §4 用 `defaults_for` 算一份(不含 EXTRA_PARM)拿去跑值層級守衛,
#   §8 再算一份並在後面接上 EXTRA_PARM ⇒ 同一份清單存在兩個地方而且**不一樣**:
#     ① `EXTRA_PARM=/tmp/x.parm` 裡寫 `ARMING_REQUIRE 0` 或 `Q_M_PWM_MIN 1100`
#        會**完全繞過** §4 的兩道守衛,而 --defaults 的後者覆蓋前者語意
#        (AP_Param.cpp:2406-2427 的 strtok_r)保證它就是最終生效值;
#     ② EXTRA_PARM 指到不存在的檔時,§4 的逐檔存在檢查也看不到它,
#        而 load_defaults_file() 少一個檔就 return false ⇒ **整批 defaults 不載入**,
#        症狀只是「參數怎麼跟我想的不一樣」,零告警。
#   ⇒ 清單只有這一個真相來源,§4 與 §8 一律呼叫這個函式,兩邊拿到逐字元相同的字串。
defaults_for() {   # defaults_for <profile> → 逗號分隔清單
  # ⚠️ PROFILE_PARM_* 的覆寫必須在**這裡**也生效,不能只在 §1 的存在檢查裡生效
  #    —— 否則「被檢查的那一份清單」和「真的送給 --defaults 的那一份」是兩份
  #    不同的東西,而值層級守衛驗錯對象時它的綠燈毫無意義。
  local p="$1" base lst pk ovf
  pk="$(profile_key "$p")"
  eval "ovf=\${PROFILE_PARM_$pk:-}"
  base="${ovf:-$(base_parm_for "$p")}"
  case "$p" in
    iris|iris_*|*copter*) lst="$base,$OVERLAY_COMMON,$OVERLAY_COPTER" ;;
    zephyr|zephyr_*)      lst="$base,$OVERLAY_COMMON" ;;
    *)                    lst="$base,$OVERLAY_COMMON,$OVERLAY_QP" ;;
  esac
  [ -n "$EXTRA_PARM" ] && lst="$lst,$EXTRA_PARM"
  printf '%s' "$lst"
}

# 用 strtok 的同一套語意把清單疊起來(後者覆蓋前者),再對**最終值**下判斷。
# 這是值層級的守衛:改上游、改下游、加旁路都免疫,而源碼層的守衛只驗「那一行長什麼樣」。
merged_val() {   # merged_val <逗號清單> <PARAM_NAME> → 最後一次出現的值(沒有就空)
  local want="$2" f line v="" oldifs="$IFS"
  local -a files=()
  # ⚠️ 逗號切檔名只能**在這裡**改 IFS。第一版把 `local IFS=','` 開在整個函式上,
  #    結果下面 `set -- $line` 的字串切分也跟著用逗號 ⇒ 每一行都被當成一個欄位、
  #    永遠比不到參數名,函式安靜地回空字串,而呼叫端只會說「讀不到 Q_M_PWM_MIN」。
  IFS=','; read -r -a files <<<"$1"; IFS="$oldifs"
  for f in "${files[@]}"; do
    [ -f "$f" ] || continue
    while IFS= read -r line || [ -n "$line" ]; do
      line="${line%$'\r'}"                                  # 容忍 CRLF 的 .parm
      line="${line#"${line%%[![:space:]]*}"}"               # 去前導空白再判註解
      case "$line" in \#*|"") continue ;; esac
      # 支援 "NAME value" 與 "NAME = value" 兩種寫法(gazebo-*.parm 用後者)
      line="${line//=/ }"
      # shellcheck disable=SC2086
      set -- $line
      if [ "${1:-}" = "$want" ]; then v="${2:-}"; fi
    done < "$f"
  done
  printf '%s' "$v"
}

QPMIN=1000; QPMAX=2000
declare -A SEEN_LIST=()
for k in $(seq 0 $((N-1))); do
  p="${PROFILES[$k]}"; lst="$(defaults_for "$p")"
  [ -n "${SEEN_LIST[$lst]:-}" ] && continue
  SEEN_LIST[$lst]=1
  IFS=',' read -r -a DEF_A <<<"$lst"
  for f in "${DEF_A[@]}"; do
    [ -f "$f" ] || die "--defaults 清單裡的 $f 不存在(profile=$p)"
  done
  say "defaults[$p]: $lst"
  # ⚠️ ARMING_REQUIRE 0 會讓 QuadPlane 的四軸馬達停在 PWM 1000,而飛機看起來一切正常。
  AR="$(merged_val "$lst" ARMING_REQUIRE)"
  if [ -n "$AR" ] && [ "$(printf '%.0f' "$AR" 2>/dev/null || echo 9)" = "0" ]; then
    die "profile '$p' 合併後的 defaults 把 ARMING_REQUIRE 設成 0 —— 這會讓 QuadPlane
     的四軸馬達停在 PWM 1000(本專案實測過的災難),而且完全沒有告警。擋下這趟。"
  fi
  case "$p" in
    iris|iris_*|*copter*|zephyr|zephyr_*) ;;
    *)
      v1="$(merged_val "$lst" Q_M_PWM_MIN)"; v2="$(merged_val "$lst" Q_M_PWM_MAX)"
      [ -n "$v1" ] && [ -n "$v2" ] || die "profile '$p' 的 defaults 讀不到 Q_M_PWM_MIN/MAX"
      QPMIN="$(printf '%.0f' "$v1")"; QPMAX="$(printf '%.0f' "$v2")"
      say "  值層級讀回: ARMING_REQUIRE='${AR:-（未設，用韌體預設）}'  Q_M_PWM_MIN/MAX=$QPMIN/$QPMAX"
      ;;
  esac
done
say "升力馬達 PWM 刻度(要與世界檔的 servo_min/max 一致):$QPMIN/$QPMAX"

# ═══════════════════════════════════════════════════════════════════════════
# §5 世界檔:產生 + 逐值讀回
# ═══════════════════════════════════════════════════════════════════════════
say "══ §5 世界檔 ══"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$HOME_LL"
[ -n "${BHDG:-}" ] || die "HOME_LL 要有四段 lat,lon,alt,hdg(目前:$HOME_LL)"

if [ -z "$WORLD" ]; then
  [ -f "$WORLD_GEN" ] || die "找不到世界產生器 $WORLD_GEN
     (那個檔由另一個工作項負責。可以先用 WORLD=/path/x.sdf 指定現成世界檔,
      §5 的逐值讀回照樣會跑。)"
  WORLD="$RUN/${WORLD_NAME_REQ}.sdf"
  GEN_ARGS=(--out "$WORLD" --world-name "$WORLD_NAME_REQ"
            --instance-base "$I0" --spacing "$SPACING"
            --lat "$BLAT" --lon "$BLON" --elevation "$BALT" --hdg "$BHDG"
            --lift-pwm-min "$QPMIN" --lift-pwm-max "$QPMAX"
            --gz-partition "$GZ_PARTITION" --model-path "$MODEL_PATH")
  # --add 的順序決定 instance 指派順序,必須與 PROFILES 的展開順序一致,
  # 否則「第 k 架」在兩邊指的是不同的飛機 —— 而埠仍然對得上,所以完全無聲。
  for item in $FLEET; do GEN_ARGS+=(--add "$item"); done
  say "產生世界檔:$WORLD_GEN ${GEN_ARGS[*]}"
  python3 "$WORLD_GEN" "${GEN_ARGS[@]}" \
    || die "世界產生器失敗(rc=$?)。它的介面若與上面的旗標不同,改本腳本 §5 的呼叫,
     **不要**改埠位表。" 1
fi
[ -f "$WORLD" ] || die "世界檔不存在:$WORLD"
PORTS_JSON="$WORLD.ports.json"
say "世界檔:$WORLD"

# ── 讀回 ①:XML 格式 ────────────────────────────────────────────────────
python3 -c 'import sys,xml.etree.ElementTree as ET; ET.parse(sys.argv[1])' "$WORLD" \
  || die "世界檔 XML 格式錯誤" 1

# ── 讀回 ②:ports.json vs 世界檔 vs 我們自己的算術,三方對帳 ───────────────
# ⚠️ 三邊算出同一個數字**不等於**正確(可以一起錯);這道對帳抓的是**分歧**。
#    正確性的錨是 SITL_cmdline.cpp:258-259 / :444-449 那兩行原始碼。
FLEET_ENV="$RUN/fleet.env"
python3 - "$WORLD" "$PORTS_JSON" "$BLAT" "$BLON" "$BALT" "$QPMIN" "$QPMAX" \
         "$GZ_PARTITION" "$(printf '%s' "${PROFILES[*]}")" > "$FLEET_ENV" <<'PY_EOF'
import hashlib, json, os, re, sys, xml.etree.ElementTree as ET
(world, pj, blat, blon, balt, qpmin, qpmax, part, prof_str) = sys.argv[1:10]
want_profiles = prof_str.split()
txt = open(world, encoding="utf-8").read()
root = ET.parse(world).getroot()
bad, note = [], []

# ---- ports.json 必須存在,而且描述的是**這一份**世界檔 --------------------
if not os.path.exists(pj):
    bad.append("找不到 %s —— 啟動器與 coordinator 要用同一份對照表,不能各算各的" % pj)
    tbl = None
else:
    tbl = json.load(open(pj, encoding="utf-8"))
    md5 = hashlib.md5(open(world, "rb").read()).hexdigest()
    if tbl.get("world_md5") and tbl["world_md5"] != md5:
        bad.append("ports.json 的 world_md5=%s ≠ 實際世界檔 md5=%s\n"
                   "        ⇒ 這份對照表是**別的**世界檔留下來的(上一趟的殘留),"
                   "照著它起飛就是把飛機接到錯的埠。"
                   % (tbl["world_md5"][:12], md5[:12]))
    if tbl.get("gz_partition") and tbl["gz_partition"] != part:
        note.append("ports.json 記的 gz_partition=%r,本趟用的是 %r"
                    % (tbl["gz_partition"], part))

vehicles = (tbl or {}).get("vehicles", [])
if tbl is not None and len(vehicles) != len(want_profiles):
    bad.append("ports.json 有 %d 架,本腳本要起 %d 架" % (len(vehicles), len(want_profiles)))

# ---- 埠算術:用 ArduPilot 的常數獨立再推一次 -----------------------------
for i, v in enumerate(vehicles):
    inst = v["instance"]
    for key, base in (("fdm_port_in", 9002), ("irlock_port", 9005)):
        exp = base + 10 * inst
        if v.get(key) != exp:
            bad.append("第 %d 架 %s=%s,但 instance=%d 推出來應該是 %d"
                       "(SITL_cmdline.cpp:258-260, :444-452)"
                       % (i, key, v.get(key), inst, exp))
    if i < len(want_profiles) and v.get("profile") != want_profiles[i]:
        bad.append("第 %d 架:ports.json 說 profile=%r,本腳本要起 %r ——\n"
                   "        --add 的順序與 FLEET 展開順序不一致,『第 k 架』在兩邊"
                   "指的是不同的飛機,而埠仍然對得上 ⇒ 完全無聲。"
                   % (i, v.get("profile"), want_profiles[i]))
    if v.get("profile", "").startswith("quadplane") and not v.get("lift_channels_checked"):
        bad.append("第 %d 架是 QuadPlane,但 ports.json 的 lift_channels_checked 是空的\n"
                   "        ⇒ 產生器那道 PWM 刻度守衛**什麼都沒檢查**。"
                   "空 list 對 QuadPlane 是紅燈,不是綠燈。" % i)

insts = [v["instance"] for v in vehicles]
if len(set(insts)) != len(insts):
    bad.append("ports.json 的 instance 有重複:%s" % insts)

# ---- 世界檔本體 ---------------------------------------------------------
ports = [int(m) for m in re.findall(r"<fdm_port_in>\s*(\d+)\s*</fdm_port_in>", txt)]
want_ports = [v["fdm_port_in"] for v in vehicles]
if vehicles and sorted(ports) != sorted(want_ports):
    bad.append("世界檔的 fdm_port_in=%s ≠ ports.json 的 %s" % (sorted(ports), sorted(want_ports)))
if len(set(ports)) != len(ports):
    bad.append("世界檔的 fdm_port_in 有重複 %s —— SO_REUSEADDR 會讓重複 bind 成功"
               "而且不報錯,後 bind 的把封包全偷走(ArduPilotPlugin.cc:227)。" % ports)

ls = re.findall(r"<lock_step>\s*(\d+)\s*</lock_step>", txt)
if len(ls) != len(ports) or any(v != "1" for v in ls):
    bad.append("lock_step 應該每架一個且都是 1,實得 %s(plugin 埠數 %d)" % (ls, len(ports)))

# ⚠️⚠️⚠️ 這道檢查第一版是 `txt.count("ArduPilotPlugin") < len(ports)`,
#   而那**在數學上不可能為真**:每個 <plugin> 標籤自己就帶 name= 與 filename=
#   兩個同名字串,加上世界檔開頭的說明註解也提到它 —— 3 架的實測值是 **9**,
#   永遠 ≥ 架數。也就是說它是一個放在「不可能為假」的位置上的檢查器,
#   本專案列管的那一類:恆綠的守衛比沒有守衛更糟,因為它會被當成證據。
#   ⇒ 改成數**真正的 plugin 元素**,而且要求**恰好等於**架數:
#     少了 ⇒ 有機體沒有 plugin(那一架完全收不到 FDM,而 world 照跑);
#     多了 ⇒ 有多餘的 plugin 在搶同一個埠(SO_REUSEADDR 讓它安靜地偷封包)。
#   ⚠️ 屬性順序兩種寫法都要吃(name 在前或 filename 在前),註解不能算進來。
npl = len(re.findall(r'<plugin\b[^>]*filename\s*=\s*"ArduPilotPlugin"', txt))
if npl != len(ports):
    bad.append("<plugin filename=\"ArduPilotPlugin\"> 有 %d 個,但 fdm_port_in 有 %d 個 ——\n"
               "        多機的寫法是把每個 model 內文(含各自 plugin)內嵌展開進同一個\n"
               "        world,只 include 最底層的裸模型;數量對不上代表有機體沒有 plugin\n"
               "        (那一架靜靜死掉而 world 照跑)或有多餘的 plugin 在搶同一個埠。"
               % (npl, len(ports)))

# 離線:model://ground_plane 本機沒有,會去 Fuel 抓而失敗(Error Code 14)。
if re.search(r"<uri>\s*model://ground_plane\s*</uri>", txt):
    bad.append("世界檔 include 了 model://ground_plane —— 本機沒有這個模型,"
               "離線會 Error Code 14。地面要內嵌。")

sc = root.find(".//spherical_coordinates")
if sc is None:
    bad.append("世界檔沒有 <spherical_coordinates> —— 沒有它就沒有共同的座標基準,"
               "而 --home 對不上是**零告警**的")
else:
    def g(tag):
        e = sc.find(tag)
        return None if e is None or e.text is None else float(e.text)
    for tag, wantv, tol in (("latitude_deg", float(blat), 1e-6),
                            ("longitude_deg", float(blon), 1e-6),
                            ("elevation", float(balt), 0.51)):
        got = g(tag)
        if got is None or abs(got - wantv) > tol:
            bad.append("世界原點 %s=%s,但 --home 給的是 %s ⇒ 座標基準會差開而且零告警"
                       % (tag, got, wantv))

# 升力馬達通道(4/5/6/7)的 PWM 刻度必須等於 Q_M_PWM_MIN/MAX。
# ⚠️ 照抄 iris 的 1100/1900 ⇒ ArduPilotPlugin.cc:1673-1675 的
#    raw_cmd=(pwm-servo_min)/(servo_max-servo_min) 會把上下各 12.5% 夾成 0/1,
#    四顆一起損失 25% 行程,而且完全沒有訊息。
mism, seen_lift = [], 0
ch_count = {}                      # 世界檔裡每個 control channel 各出現幾次
for m in re.finditer(r'<control\s+channel\s*=\s*"(\d+)"\s*>(.*?)</control>', txt, re.S):
    ch, body = int(m.group(1)), m.group(2)
    ch_count[ch] = ch_count.get(ch, 0) + 1
    if ch > 15:
        bad.append("control channel=%d 超出 0-15" % ch)
    if ch not in (4, 5, 6, 7):
        continue
    seen_lift += 1
    smin = re.search(r"<servo_min>\s*([\d.]+)\s*</servo_min>", body)
    smax = re.search(r"<servo_max>\s*([\d.]+)\s*</servo_max>", body)
    lo = float(smin.group(1)) if smin else 1100.0   # plugin 的內建預設就是 1100/1900
    hi = float(smax.group(1)) if smax else 1900.0
    if abs(lo - float(qpmin)) > 0.5 or abs(hi - float(qpmax)) > 0.5:
        mism.append("channel=%d servo_min/max=%g/%g" % (ch, lo, hi))
if mism:
    bad.append("升力通道的 PWM 刻度與 Q_M_PWM_MIN/MAX=%s/%s 對不上:%s"
               % (qpmin, qpmax, "; ".join(mism)))
if seen_lift == 0 and any(p.startswith("quadplane") for p in want_profiles):
    bad.append("機隊裡有 QuadPlane,但世界檔完全沒有 channel 4-7 的 control ——"
               "升力馬達沒接上(ArduPlane 把 motor1..4 指到 CH_5..CH_8,"
               "quadplane.cpp:667-671;SDF 的 channel 是 0-based ⇒ 4/5/6/7)。")

# ⚠️⚠️⚠️ 上面那條 `seen_lift == 0` 是**必要但遠遠不充分**的:
#   seen_lift 是整份世界檔的**總數**,四架 QuadPlane 裡只有一架接上升力通道時
#   它是 4(不是 0)⇒ 綠燈,而另外三架的四顆升力馬達整趟收不到任何指令。
#   那三架會**乖乖地起飛失敗**,而所有埠、所有 log 錨點、sim_time 全都正常。
#   ⇒ 逐通道對帳:ports.json 的 lift_channels_checked 說每一架要檢查哪些通道,
#     把它累加成「每個 channel 應該出現幾次」,再和世界檔實際出現的次數比。
#   ⚠️ lift_channels_checked 是空 list 時上面已經另有一條紅燈(空 ≠ 綠),
#     這裡不會因為它空掉而變成沒東西可比。
want_ch = {}
for v in vehicles:
    for c in (v.get("lift_channels_checked") or []):
        want_ch[int(c)] = want_ch.get(int(c), 0) + 1
for c in sorted(want_ch):
    if ch_count.get(c, 0) != want_ch[c]:
        bad.append("升力通道 channel=%d 在世界檔裡出現 %d 次,但 ports.json 的"
                   " lift_channels_checked 累加起來應該是 %d 次 ——"
                   "        ⇒ 有機體的升力馬達根本沒接上(而它會安靜地起飛失敗:"
                   "埠、log 錨點、sim_time 全部正常)。"
                   % (c, ch_count.get(c, 0), want_ch[c]))

for n in note:
    print("# note: " + n, file=sys.stderr)
print("[world] fdm=%s  lock_step=%s  plugin×%d  升力通道×%d  %s"
      % (sorted(ports), sorted(set(ls)) or "(無)", npl, seen_lift,
         "ports.json ✔" if tbl else "ports.json ✗"), file=sys.stderr)
if bad:
    print("[world] ✗ 逐值讀回失敗:", file=sys.stderr)
    for b in bad:
        print("      • " + b, file=sys.stderr)
    sys.exit(1)
print("[world] ✔ 逐值讀回通過(ports.json / 世界檔 / 韌體算術 三方一致)", file=sys.stderr)

# ---- 輸出給 shell ------------------------------------------------------
wn = (tbl or {}).get("world_name") or (root.find("world").get("name") if root.find("world") is not None else "")
print('WORLD_NAME_ACTUAL="%s"' % wn)
print("FLEET_N=%d" % len(vehicles))
for i, v in enumerate(vehicles):
    print('VEH_PROFILE_%d="%s"' % (i, v.get("profile", "")))
    print('VEH_NAME_%d="%s"' % (i, v.get("name", "uav%d" % i)))
    print("VEH_INSTANCE_%d=%d" % (i, v["instance"]))
    print("VEH_FDM_%d=%d" % (i, v["fdm_port_in"]))
    print("VEH_SYSID_%d=%d" % (i, v.get("sysid_suggested", v["instance"] + 1)))
PY_EOF
# ⚠️ 用 $? 而不是 PIPESTATUS:上面是簡單命令(只有重導向、沒有管線),
#    PIPESTATUS 在這裡雖然也對,但只要有人日後加一段 `| tee` 就會靜靜地取到別人的碼。
RB_RC=$?
{ [ "$RB_RC" -eq 0 ] && [ -s "$FLEET_ENV" ]; } \
  || die "世界檔與我們的埠位/參數對不上(上面已逐項列出;讀回程式 rc=$RB_RC)" 1
# shellcheck disable=SC1090
. "$FLEET_ENV"
[ "${FLEET_N:-0}" -eq "$N" ] || die "fleet.env 的機數 ${FLEET_N:-0} ≠ N=$N"
say "world name(讀回)= $WORLD_NAME_ACTUAL"

# ── 讀回 ③:gz 自己的 SDF 驗證(需要 SDF_PATH 才解析得了 model://)────────
if gz sdf -k "$WORLD" >"$RUN/sdf_check.log" 2>&1; then
  say "gz sdf -k:Valid"
else
  warn "gz sdf -k 沒過,細節見 $RUN/sdf_check.log(最後 5 行):"; tail -5 "$RUN/sdf_check.log" >&2
  die "世界檔 SDF 驗證失敗" 1
fi

warn "座標基準提醒:世界原點 elevation=$BALT m,但地面在世界 z=0
     ⇒ Gazebo 的『海平面』其實是 $BALT m AMSL。這是已知未解問題,先照 $BALT 做。"

# ═══════════════════════════════════════════════════════════════════════════
# §6 埠位前置檢查(以 ports.json 的 instance 為準)
# ═══════════════════════════════════════════════════════════════════════════
say "══ §6 埠位前置檢查 ══"
INST_LIST=""; for k in $(seq 0 $((N-1))); do
  eval "i=\$VEH_INSTANCE_$k"; INST_LIST="${INST_LIST:+$INST_LIST,}$i"
done
[ "$TCP_BASE" != "5760" ] || die "TCP_BASE 不可以是 5760(SITL_cmdline.cpp:435 的守衛
     是比值的:顯式給 5760 之後 -I 還是會加 instance*10,我們就再也不知道它 listen 在哪)"
MAP="$(python3 "$PROBE" --instances "$INST_LIST" --tcp-base "$TCP_BASE" --print-map)" \
  || die "取埠位表失敗(--print-map)"
eval "$MAP"
read -r -a FDM_A  <<<"${VTOL_FDM_PORTS:?埠位表沒有 VTOL_FDM_PORTS}"
read -r -a TCPB_A <<<"${VTOL_TCP_BASES:?埠位表沒有 VTOL_TCP_BASES}"
# 對帳:probe 自己推的 fdm 埠必須等於 ports.json 給的
for k in $(seq 0 $((N-1))); do
  eval "want=\$VEH_FDM_$k"
  [ "${FDM_A[$k]}" = "$want" ] || die "第 $k 架的 fdm 埠分歧:probe=${FDM_A[$k]} ports.json=$want"
  eval "prof=\$VEH_PROFILE_$k; inst=\$VEH_INSTANCE_$k; sid=\$VEH_SYSID_$k"
  # ⚠️ sysid 由 ports.json 建議成 instance+1(gz_vtol_world.py:1482)。MAVLink 的
  #    合法範圍是 1-255,而 0 是廣播、255 慣例上是 GCS 自己。I0 給大一點就會
  #    悄悄跑出範圍,而症狀是「coordinator 看不到某一架」——
  #    那和埠錯、和串流沒開起來的表徵完全一樣,肉眼分不出是哪一個。
  { [ "$sid" -ge 1 ] && [ "$sid" -le 254 ]; } 2>/dev/null \
    || die "uav$k 的 sysid=$sid 不在 1-254(instance=$inst ⇒ instance+1)。
     0 是 MAVLink 廣播、255 慣例上是 GCS ⇒ 這一架在 coordinator 眼裡會消失,
     而表徵和『埠錯』『串流沒開』一模一樣。改 I0 讓 instance+1 落回範圍內。"
  say "uav$k  profile=$prof  I=$inst  sysid=$sid  fdm=${FDM_A[$k]}  SERIAL0=${TCPB_A[$k]}"
done
say "⚠️ SITL 的 TCP 段用 --base-port 顯式挪到 $TCP_BASE 起,**不用**韌體預設的 5760+10i"
say "   (I0=$I0 的預設會是 $((5760+10*I0))+,正好撞上 sar_swarm 的 MAVROS 5860 帶)"

# ⚠️ 離開碼要分開歸因:probe 的契約是 0 乾淨 / 1 髒 / 2 用法或內部錯 / 3 自檢壞了。
#   第一版不管收到幾,都印「埠位不乾淨,上面已指認佔用者」,而組態打錯時
#   根本沒有佔用者可以指認 ⇒ 人會去找一個不存在的兇手。
#   本專案列管過這一類:錯誤歸因會驅動錯誤的修復。
python3 "$PROBE" --instances "$INST_LIST" --tcp-base "$TCP_BASE"
PROBE_RC=$?
case "$PROBE_RC" in
  0) ;;
  1) die "埠位不乾淨(上面已指認佔用者)。**不要**隨手換埠段就跑,
     先確認那是不是別人正在跑的航次" 1 ;;
  2) die "埠位檢查的**組態**有問題(rc=2,不是埠髒)—— 上面那一行是它的說法。
     常見原因:instance 或 TCP_BASE 推出來的埠超出 16 bit,或某一架的
     base_port 剛好等於韌體預設 5760。改 I0 / TCP_BASE,不要去找佔用者。" 2 ;;
  *) die "埠位檢查自己壞了(rc=$PROBE_RC)—— 它的綠燈與紅燈都不可信,擋下這趟" 3 ;;
esac

if [ "$DRY_RUN" = "1" ]; then
  say "══ DRY_RUN=1:檢查與產檔都已完成,不起任何行程 ══"; say "世界檔:$WORLD"; exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
# §7 起 gz
# ═══════════════════════════════════════════════════════════════════════════
say "══ §7 起 gz ══"
GZ_LOG="$RUN/gz.log"; GZ_PID_F="$RUN/gz.pid"
# ⚠️⚠️⚠️ 先刪掉上一趟的殘留,**這是閘門正確性的前提**。
#   $RUN 跨趟保留(down.sh 刻意不清),而 pid 檔是 `echo $$` 寫在 exec 之前的。
#   若 setsid/bash 這一段因為任何理由沒走到 exec(cd 失敗、binary 不見、fork 失敗),
#   舊的 gz.pid / gz.log 就會原封不動留著,於是:
#     * kill -0 <舊 pid> 在 pid 被回收後**會成功** ⇒ 「gz 起來了」是假綠燈;
#     * §9 逐機閘門 grep 到的是**上一趟**的 log ⇒ G1/G2/G4 全綠而這一趟根本沒跑。
#   這正是本專案列管的「檢查器響到錯的東西」。刪檔是唯一能讓它一定響對的做法。
rm -f "$GZ_LOG" "$GZ_PID_F"
GZ_ARGS=(sim -r -v 2 "$WORLD")
[ "$HEADLESS" = "1" ] && GZ_ARGS=(sim -s -r -v 2 "$WORLD")
# setsid:讓 gz 離開本終端的 session,Ctrl+C(SIGINT 送整個前景 group)與關終端
# (SIGHUP)都不會順手殺掉它。`echo $$` 在 exec 之前寫下的就是被 exec 取代的那個 pid。
GZ_LOG="$GZ_LOG" GZ_PID_F="$GZ_PID_F" setsid bash -c \
  'echo $$ > "$GZ_PID_F"; exec gz "$@" >"$GZ_LOG" 2>&1' _ "${GZ_ARGS[@]}" &
sleep 1
# pid 檔剛剛才被刪掉 ⇒ 這裡讀得到就一定是這一趟寫的,讀不到就是根本沒起來。
[ -f "$GZ_PID_F" ] || die "gz 的 pid 檔沒被寫出來 —— setsid/bash 那一段沒走到 exec,見 $GZ_LOG" 4
GZ_PID="$(cat "$GZ_PID_F" 2>/dev/null || echo)"
{ [ -n "$GZ_PID" ] && kill -0 "$GZ_PID" 2>/dev/null; } || die "gz sim 起不來,見 $GZ_LOG" 4
say "gz pid=$GZ_PID  log=$GZ_LOG  partition=$GZ_PARTITION"

is_ours() {   # is_ours <pid> —— environ 在 exec 時就固定,偽裝不了,是最硬的指認
  [ -r "/proc/$1/environ" ] || return 1
  tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | grep -qxF "GZ_VTOL_RUN_ID=$GZ_VTOL_RUN_ID"
}
# ⚠️⚠️ 回傳**所有**佔用者的 pid,不是 head -1。
#   SO_REUSEADDR 之下同一個 UDP 埠可以有兩個以上的 binder,ss 會列成**兩行**,
#   而封包只給最後 bind 的那一個(ArduPilotPlugin.cc:227 / SocketUDP.cc:71-75)。
#   只取第一行的話,「我們的 gz 有 bind」會蓋掉「還有別人也在同一個埠上」——
#   那正是整支 gz_port_probe.py 在防的那一個失效,不能在這裡又漏回去。
#   ⚠️ 欄位取第 4 欄(Local Address:Port);probe 的檔頭記著第一版誤用 Peer 欄
#     ⇒ 被佔的 9002/9012/9022/9032 一個都沒列出來的教訓。
port_owner_pids() {
  ss -lunp 2>/dev/null | awk -v p=":$1\$" '$4 ~ p {print}' \
    | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u
}
# ⚠️ 這裡**可以**用 ss:§6 已經證明過這些埠原本是空的,所以現在出現的 binder
#    只可能是我們剛起的 gz;再加一道 environ 比對確認。
#    「埠有人 bind」單獨拿來當**乾淨與否**的判準才是不行的(SO_REUSEADDR)。
say "等 gz bind 每一個 fdm 埠(上限 ${GZ_WAIT_S}s)…"
deadline=$(( $(date +%s) + GZ_WAIT_S ))
for k in $(seq 0 $((N-1))); do
  p="${FDM_A[$k]}"
  while :; do
    owners="$(port_owner_pids "$p")"
    if [ -n "$owners" ]; then
      # **每一個**佔用者都必須是本趟的行程。只要多出一個別人的,
      # 「封包會落到誰手上」這件事就已經失控,而且兩邊的 bind 都不會報錯。
      alien=""
      while read -r owner; do
        [ -n "$owner" ] || continue
        is_ours "$owner" || alien="${alien:+$alien }$owner"
      done <<<"$owners"
      if [ -z "$alien" ]; then
        say "  uav$k  fdm $p ← pid $(echo $owners)(全部確認是本趟的行程)"
        break
      fi
      die "fdm 埠 $p 上有**不是**本趟起的行程在佔著:pid $alien
     §6 剛檢查時它還是空的 ⇒ 有人在這中間插進來,而 SO_REUSEADDR 讓兩邊的 bind
     都成功、都不報錯,封包只給最後 bind 的那一個。
     中止,不要覆蓋別人的航次。" 1
    fi
    [ "$(date +%s)" -lt "$deadline" ] || die "等 gz bind fdm 埠 $p 逾時,見 $GZ_LOG" 4
    sleep 0.5
  done
done

# ═══════════════════════════════════════════════════════════════════════════
# §8 起 SITL(逐架挑 binary 與參數底座)
# ═══════════════════════════════════════════════════════════════════════════
say "══ §8 起 SITL(${N} 架)══"
case "$SERIAL0_MODE" in
  nowait) SERIAL0="tcp:0" ;;        # 不等 GCS(見檔頭「SERIAL0 的 accept 閘門」)
  wait)   SERIAL0="tcp:0:wait" ;;
  *)      die "SERIAL0_MODE 只能是 nowait 或 wait(目前 $SERIAL0_MODE)" ;;
esac
say "SERIAL0 模式:$SERIAL0_MODE → --serial0 $SERIAL0"

for k in $(seq 0 $((N-1))); do
  eval "prof=\$VEH_PROFILE_$k; I=\$VEH_INSTANCE_$k; SYSID=\$VEH_SYSID_$k"
  FDM="${FDM_A[$k]}"; BP="${TCPB_A[$k]}"
  PK="$(profile_key "$prof")"
  eval "OVB=\${PROFILE_BIN_$PK:-}"
  BIN="${OVB:-$(sitl_binary_for "$prof")}"
  # ⚠️ EXTRA_PARM 已經在 defaults_for() 裡疊好了(見 §4 的註解)。
  #    **不要**在這裡再接一次 —— 那會讓 §8 送出去的清單和 §4 驗過的那份不同,
  #    而值層級守衛驗的是「不是實際送出去的東西」時,它的綠燈毫無意義。
  DEFS="$(defaults_for "$prof")"
  WD="$RUN/uav$k"; mkdir -p "$WD"
  LOGF="$WD/sitl.log"; PIDF="$WD/sitl.pid"
  # ⚠️⚠️⚠️ 同 §7:先刪上一趟的殘留。`exec "$BIN" … >"$LOGF"` 是在 exec 那一刻才截斷
  #   檔案的,而 `echo $$ > "$PIDF"` 在它之前 ⇒ 兩者之間有一個窗口,
  #   而 exec 失敗時 log 根本不會被截斷。留著舊檔的後果:
  #     * §9 第一個等待迴圈 grep 到**上一趟**的 "JSON received:" ⇒ 立刻通過;
  #     * G1/G2 讀回的是上一趟的埠 —— 而上一趟用的就是同一組埠 ⇒ **恆綠**;
  #     * G5 的成長檢查在一個不再被寫入的舊檔上永遠是 0 成長 ⇒ 也是恆綠。
  #   也就是說,不刪檔的話 §9 這五道閘門在「SITL 根本沒起來」時**全部報綠**。
  rm -f "$LOGF" "$PIDF"
  ARGS=(--model JSON --speedup "$SPEEDUP" -I "$I" --sysid "$SYSID"
        --home "$HOME_LL" --defaults "$DEFS"
        --sim-address 127.0.0.1 --sim-port-out "$FDM"
        --base-port "$BP" --serial0 "$SERIAL0")
  # -w:每趟清 eeprom。本專案的硬性要求是「不接受吃到舊參數」;留著 eeprom 就是
  #    留著一條「--defaults 改了但行為沒變」的靜默路徑。
  [ "$WIPE" = "1" ] && ARGS=(-w "${ARGS[@]}")
  printf '%s %s\n' "$BIN" "${ARGS[*]}" > "$WD/cmdline.txt"
  WD="$WD" LOGF="$LOGF" PIDF="$PIDF" BIN="$BIN" setsid bash -c \
  # 
  # 2026-09-10 A/B (same binary, same flags, same gz):
  #   setsid bash -c '... exec arduplane ...'  => SITL dies as soon as this script
  #       exits. Its log stops at the 2nd "Loaded defaults" with no error at all.
  #   setsid bash <script that calls arduplane>  => survives (measured many minutes).
  #   The only difference is exec: with exec, arduplane itself becomes the session
  #   leader. Keep a bash parent in the session and it lives. gz looks immune only
  #   because "gz sim" forks server/gui children; the ruby shell is what dies.
  #   WARNING: this failure is maximally misleading. The per-aircraft gate in
  #   section 9 PASSES (the process really is alive at that moment) and the death
  #   happens seconds later, so it reads as "lock_step froze" or "someone killed it".
  #   Note: a log that stops at "Loaded defaults" does NOT mean dead - ArduPilot is
  #   simply quiet after boot. Judge with pgrep or with SERIAL0 listening.
  # Not using exec also lets us capture the exit status into the log tail.
  WD="$WD" LOGF="$LOGF" PIDF="$PIDF" BIN="$BIN" setsid bash -c \
    'cd "$WD" || exit 90
     "$BIN" "$@" >"$LOGF" 2>&1 &
     child=$!
     echo "$child" > "$PIDF"
     wait "$child"; rc=$?
     printf "\n[wrapper] SITL_EXIT=%s (128+n means signal n: 139=SEGV 143=TERM 137=KILL)\n" "$rc" >> "$LOGF"' \
    _ "${ARGS[@]}" &
  sleep 0.5
  [ -f "$PIDF" ] || die "uav$k pid file missing - setsid/bash never reached the launch line (cd $WD failed?), see $LOGF" 4
  say "uav$k  $prof  $(basename "$BIN")  I=$I sysid=$SYSID fdm→$FDM base_port=$BP pid=$(cat "$PIDF" 2>/dev/null)"
done

# ═══════════════════════════════════════════════════════════════════════════
# §9 逐機驗活 —— sim_time 前進**不夠**,必須每一架分別過關
# ═══════════════════════════════════════════════════════════════════════════
say "══ §9 逐機驗活 ══"
cnt() {   # grep -c 在 0 筆時回 1;套上 pipefail 會毒死整條管線,所以包起來
  local n; n="$(grep -c -- "$1" "$2" 2>/dev/null)" || n=0; printf '%s' "${n:-0}"
}

# "JSON received:" 只在收到的**欄位集合改變**時才印(SIM_JSON.cpp:362-378)
# ⇒ 它是「第一包到了」的證據,**不是**持續心跳。持續性靠 G5 的成長檢查。
deadline=$(( $(date +%s) + SITL_WAIT_S ))
for k in $(seq 0 $((N-1))); do
  LOGF="$RUN/uav$k/sitl.log"
  while [ "$(cnt 'JSON received:' "$LOGF")" = "0" ]; do
    PID="$(cat "$RUN/uav$k/sitl.pid" 2>/dev/null || echo)"
    if [ -z "$PID" ] || ! kill -0 "$PID" 2>/dev/null; then
      warn "uav$k 的 SITL 已經死了(pid=${PID:-?}),log 尾巴:"; tail -15 "$LOGF" >&2; break
    fi
    [ "$(date +%s)" -lt "$deadline" ] || break
    sleep 0.5
  done
done

# 取樣一:各機的 "No JSON sensor message received" 累計次數
# (SIM_JSON.cpp:326-330:連續 >1000 ms 收不到 FDM 就印一次並重送 servo)
declare -a S1=()
for k in $(seq 0 $((N-1))); do
  S1[$k]="$(cnt 'No JSON sensor message received' "$RUN/uav$k/sitl.log")"
done
say "等 ${LIVE_WINDOW_S}s 再取樣一次 —— 要看的是**有沒有成長**,不是有沒有出現"
say "  (開機那幾秒 gz 還沒 bind 好,出現幾次是正常的)"
sleep "$LIVE_WINDOW_S"

FAILED=()
for k in $(seq 0 $((N-1))); do
  eval "prof=\$VEH_PROFILE_$k; I=\$VEH_INSTANCE_$k"
  FDM="${FDM_A[$k]}"; BP="${TCPB_A[$k]}"
  LOGF="$RUN/uav$k/sitl.log"; PID="$(cat "$RUN/uav$k/sitl.pid" 2>/dev/null || echo)"
  problems=()

  # G0 行程還在
  { [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; } \
    || problems+=("SITL 行程不在了(pid=${PID:-?})")

  # G1 值層級讀回:FDM 埠。SITL 自己印出它要送去哪(SIM_JSON.cpp:99),直接和
  #    ports.json/probe 算出來的比。這就是「埠與 instance 綁死」那條事實的閘門。
  got_fdm="$(grep -o 'JSON control interface set to 127\.0\.0\.1:[0-9]*' "$LOGF" 2>/dev/null \
             | tail -1 | sed 's/.*://')"
  if [ -z "$got_fdm" ]; then
    problems+=("log 裡找不到 'JSON control interface set to ...' —— SITL 沒走到 JSON 後端")
  elif [ "$got_fdm" != "$FDM" ]; then
    problems+=("FDM 埠讀回 $got_fdm ≠ 期望 $FDM(instance=$I ⇒ 9002+10*$I)")
  fi

  # G2 值層級讀回:SERIAL0 的 listen 埠(UARTDriver.cpp:392 印 'bind port N for SERIAL0')
  got_bp="$(grep -o "bind port [0-9]* for SERIAL0" "$LOGF" 2>/dev/null | tail -1 | awk '{print $3}')"
  if [ -z "$got_bp" ]; then
    problems+=("log 裡找不到 'bind port ... for SERIAL0'")
  elif [ "$got_bp" != "$BP" ]; then
    problems+=("SERIAL0 讀回 $got_bp ≠ 期望 $BP —— --base-port 沒吃進去")
  fi

  # G3 讀回「我們對 SERIAL0 閘門的選擇」真的生效了
  if [ "$SERIAL0_MODE" = "nowait" ] && [ "$(cnt 'Waiting for connection' "$LOGF")" != "0" ]; then
    problems+=("選了 nowait 卻出現 'Waiting for connection ....' ⇒ --serial0 tcp:0 沒生效,"
               "SITL 正阻塞在 accept()(UARTDriver.cpp:415-427),表徵和收不到 FDM 一樣")
  fi

  # G4 第一個 FDM 封包到過(證明 arduPilotOnline 曾經為 true)
  [ "$(cnt 'JSON received:' "$LOGF")" != "0" ] \
    || problems+=("log 裡沒有 'JSON received:' —— 從來沒收到第一個 FDM 封包;"
                  "這一架會**靜靜死掉而 world 照跑**(ArduPilotPlugin.cc:1204-1213)")

  # G5 現在還在收(充分性那一半)
  s2="$(cnt 'No JSON sensor message received' "$LOGF")"
  if [ "$s2" -gt "${S1[$k]}" ]; then
    problems+=("最近 ${LIVE_WINDOW_S}s 內 'No JSON sensor message' 從 ${S1[$k]} 長到 $s2"
               " ⇒ FDM 鏈路**現在**是斷的(gz 沒送、或封包被別人偷走)")
  fi

  if [ "${#problems[@]}" -eq 0 ]; then
    say "  ✔ uav$k [$prof] I=$I fdm=$got_fdm SERIAL0=$got_bp 活著"
  else
    FAILED+=("$k")
    printf '[vtol] ✗ uav%d [%s] I=%s 沒過:\n' "$k" "$prof" "$I" >&2
    for pb in "${problems[@]}"; do printf '        • %s\n' "$pb" >&2; done
    printf '        log 尾巴(%s):\n' "$LOGF" >&2
    tail -8 "$LOGF" 2>/dev/null | sed 's/^/          /' >&2
  fi
done

# 世界層級:sim_time。**必要但不充分**,只印出來當參考,不參與判決。
# ⚠️⚠️ `gz topic -e -n 1` 在**沒有任何發布者**時會一直等下去(它等的是「收到 1 則」,
#   不是「查一次」)。世界名打錯、GZ_PARTITION 不合、或整個 world 被 lock_step 凍住
#   —— 這三種情況它都不會回來。而這一行在整支腳本的最後面,前面所有閘門都已經過了,
#   於是失效的表徵會變成「腳本卡在最後不結束」,看起來像 gz 掛了。
#   它只是**參考資訊、不是閘門** ⇒ 一定要有 timeout,而且逾時只印「讀不到」。
T1="$(timeout 5 gz topic -e -t "/world/$WORLD_NAME_ACTUAL/stats" -n 1 2>/dev/null \
      | grep -A3 'sim_time' | grep -o 'sec: *[0-9]*' | head -1)"
say "參考(非閘門):/world/$WORLD_NAME_ACTUAL/stats sim_time = ${T1:-讀不到}"
say "  ⚠️ 沒收到第一包的機體會靜靜死掉而 world 照跑;反過來一架斷線會凍住整個 world"
say "     而且沒有 CPU 尖峰(是 sleep 不是 busy-spin)。判決以上面的逐機閘門為準。"

# ═══════════════════════════════════════════════════════════════════════════
# §10 state 檔(down / coordinator 共用同一份真相)
# ═══════════════════════════════════════════════════════════════════════════
{
  echo "# 由 gz_vtol_up.sh 產生 —— $(date '+%F %T')"
  echo "GZ_VTOL_RUN_ID=\"$GZ_VTOL_RUN_ID\""
  echo "GZ_PARTITION=\"$GZ_PARTITION\""
  echo "VTOL_N=$N"
  echo "VTOL_FLEET=\"$FLEET\""
  echo "VTOL_I0=$I0"
  echo "VTOL_TCP_BASE=$TCP_BASE"
  echo "VTOL_INSTANCES=\"$VTOL_INSTANCES\""
  echo "VTOL_FDM_PORTS=\"$VTOL_FDM_PORTS\""
  echo "VTOL_TCP_BASES=\"$VTOL_TCP_BASES\""
  echo "VTOL_HOME=\"$HOME_LL\""
  echo "VTOL_WORLD=\"$WORLD\""
  echo "VTOL_WORLD_NAME=\"$WORLD_NAME_ACTUAL\""
  echo "VTOL_PORTS_JSON=\"$PORTS_JSON\""
  echo "VTOL_RUN=\"$RUN\""
  echo "VTOL_SERIAL0_MODE=\"$SERIAL0_MODE\""
  echo "VTOL_GZ_PID=$GZ_PID"
  for k in $(seq 0 $((N-1))); do
    eval "prof=\$VEH_PROFILE_$k"
    echo "VTOL_PROFILE_$k=\"$prof\""
    echo "VTOL_SITL_PID_$k=$(cat "$RUN/uav$k/sitl.pid" 2>/dev/null || echo 0)"
  done
} > "$STATE"
say "state → $STATE"

if [ "${#FAILED[@]}" -ne 0 ]; then
  printf '[vtol] ✗✗ 逐機驗活失敗的是:' >&2
  for k in "${FAILED[@]}"; do printf ' uav%s' "$k" >&2; done
  printf '\n[vtol]    (**逐架**報出來,不是只報總結 —— 一架死掉會凍住整個 world,\n' >&2
  printf '[vtol]     而總結欄位在那種情況下看起來完全正常。)\n' >&2
  printf '[vtol]    收乾淨:bash %s/gz_vtol_down.sh\n' "$SR" >&2
  exit 4
fi

say "✔ ${N} 架全部通過逐機驗活。"
printf '[vtol]   coordinator 接 SERIAL0:'
for k in $(seq 0 $((N-1))); do printf 'tcp:127.0.0.1:%s ' "${TCPB_A[$k]}"; done; printf '\n'
say "  ⚠️ coordinator 連上後必須**無條件**請求串流(SET_MESSAGE_INTERVAL /"
say "     REQUEST_DATA_STREAM)並斷言真的收到:ArduPlane 預設 POSITION/EXTRA1 只有 1 Hz,"
say "     ArduCopter 預設**全部 0**(GCS_MAVLink_Parameters.cpp:49-59 / :60-71)。"
say "     串流速率參數已改名成 MAV0_/MAV1_/MAV2_*,舊的 SR0_* 會被韌體靜默略過。"
say "  停止:bash $SR/gz_vtol_down.sh"
exit 0
