#!/usr/bin/env bash
# swarm_formation_up.sh — 一鍵開 6 台固定翼「群飛編隊」完整 ROS2 堆疊。
#
# 角色：UAV1=長機(master) / UAV2-5=僚機(wingman) / UAV6=備援(backup)
# 堆疊：6×SITL(直接 binary) → MAVProxy fan-out → 6×MAVROS → sar_swarm 編隊節點
#   SITL 5760+10i ─┬─ MAVROS   tcpin 5860+10i（FCU_TCP_BASE=5860）
#                  ├─ AeroPlan tcpin 6760+10i（嵌入式蜂群監看 base_port 6760）
#                  └─ node     tcpin 6960+10i
#
# 為什麼直接跑 arduplane binary：本 WSL 的 sim_vehicle 會把 arduplane 包進 xterm，
# xterm 載不到字型 → SITL 當場死掉（見專案記憶 wsl-sitl-swarm-bridge）。
#
# 編隊型態（FORMATION）：
#   v          雁型 V（固定 slot；僚機排長機後左/後右，V 隨長機航向旋轉）＝預設
#   biomimetic 仿生 V（相鄰機追蹤，去中心 push-pull）
#
# 用法：
#   bash swarm_formation_up.sh                       # 蜂群原點、雁型V、runway、自動 arm+起飛+巡邏
#   FORMATION=biomimetic bash swarm_formation_up.sh  # 仿生 V
#   LAUNCH=catapult bash ...                         # 彈射起飛（frame plane-catapult）
#   LOC=HUWEI bash ...                               # 生在虎尾（自動關巡邏；僚機仍繞長機編隊）
#   PATROL=0 bash ...                                # 長機不自動巡邏（GCS/操作員飛）；僚機仍編隊
#   MANUAL=1 bash ...                                # 不自動 arm/起飛（操作員全手動）
#   RAFT=1 bash ...                                  # +健康優先 Raft 故障切換
#   COUNT=4 bash ...                                 # 只開前 4 台（輕量測試）
#
# 故障切換示範：pkill -f master_node → 備援 UAV6 於 ~2.5s 接手，僚機重綁新長機。
# 停止：bash swarm_stop.sh
#   或  pkill -f 'arduplane|mavproxy|mavros_node|master_node|wingman_node|backup_node'
#
# ⚠️ 不可 set -u —— source ROS2 setup.bash 會參照未綁定變數而中止整個腳本。
set -eo pipefail

# ── 路徑 ────────────────────────────────────────────────────────────
AP="$HOME/ardupilot/build/sitl/bin/arduplane"
BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/fixwing_ws/install/setup.bash"
export PATH="$HOME/.local/bin:$PATH"

# ── 出生地點預設（與 AeroPlan Studio config/spawn_presets.py 同址）──────
# ⚠️ 蜂群原點 = sar_failover_nodes.launch.py 內硬編 ORIGIN(22.60,120.00)。
#    長機的 auto_patrol 巡邏方框錨定在此原點；生成點若不在此原點，開 auto_patrol
#    會讓長機飛向 ~120km 外 → 本腳本在非原點時自動關閉 auto_patrol。
#    （僚機編隊是「相對長機即時位置」，任何生成點都能正常編隊，不受此限。）
SWARM_ORIGIN="22.60,120.00,0,90"
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    SWARM|"")   echo "$SWARM_ORIGIN" ;;                 # 蜂群原點（節點 ORIGIN 同址）
    HUWEI)      echo "23.702732,120.419333,0,0"  ;;     # 虎尾科大 / 專案預設
    TAOYUAN|RCTP|TPE) echo "25.0777,121.2325,32,52" ;;  # 桃園機場 跑道 05
    *)          echo "$1" ;;                            # 已是 lat,lon,alt,hdg
  esac
}
LOC_RAW="${LOC:-SWARM}"
LOC="$(resolve_loc "$LOC_RAW")"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"

N="${COUNT:-6}"                     # 台數（1..6）
SPACING="${SPACING:-30}"           # 縱列起飛間距 [m]（沿跑道排一列）

# ── 編隊型態 → use_biomimetic ───────────────────────────────────────
case "$(echo "${FORMATION:-v}" | tr '[:upper:]' '[:lower:]')" in
  v|vee|wedge)         BIO=false ;;
  biomimetic|bio|swarm) BIO=true ;;
  *) echo "[formation] 未知 FORMATION='$FORMATION'，改用 v"; BIO=false ;;
esac

# ── 起飛型態 → SITL model / 節點 launch_type ────────────────────────
case "$(echo "${LAUNCH:-runway}" | tr '[:upper:]' '[:lower:]')" in
  catapult) MODEL="plane-catapult"; LT="catapult" ;;
  *)        MODEL="plane";          LT="runway"   ;;
esac

# ── 自動化旗標 ──────────────────────────────────────────────────────
# MANUAL=1：節點不 arm、不起飛（待命常駐），起飛由操作員觸發（swarm_takeoff.sh
#           或 GCS）；飛過 guided_min_alt(15m) 後節點自動接手 GUIDED 編隊。
# PATROL 與 MANUAL 各自獨立：MANUAL=1 PATROL=1 → 你手動起飛後長機自動帶隊巡邏。
# origin 已隨 LOC 傳給節點（見 stage 4 的 origin_lat/lon/heading_deg），巡邏方框
# 錨定在「生成點」而非硬編 22.60,120.00 → 任何 LOC 都能安全 auto_patrol。
if [ "${MANUAL:-0}" = "1" ]; then
  AA=false; AT=false
  PATROL="${PATROL:-0}"
else
  AA=true;  AT=true
  PATROL="${PATROL:-1}"
fi
if [ "$PATROL" = "1" ]; then AP_PATROL=true; else AP_PATROL=false; fi
RAFT_ARG="${RAFT:-0}"; [ "$RAFT_ARG" = "1" ] && RAFT_ARG=true || RAFT_ARG=false

# ── 編隊位置約束（速度控制）可調參數 ─────────────────────────────────
# 「長機飛太快等僚機、僚機努力追上」的行為調校旋鈕（皆可用 env 覆寫）：
#   NOMINAL_SPEED  隊形成立後巡航空速（預設 14）
#   WAIT_SPEED     有僚機落後時長機放慢等的空速（預設 10；越小＝越肯等）
#   FORMED_RADIUS  僚機距 slot < 此(m) 算到位（預設 250；越小＝要求越嚴、長機更常等）
#   CATCH_SPEED    僚機落後時的追趕空速（預設 20；越大＝追越兇）
#   CATCH_RADIUS   僚機距 slot > 此(m) 用全速追（預設 400）
NOMINAL_SPEED="${NOMINAL_SPEED:-14.0}"
WAIT_SPEED="${WAIT_SPEED:-10.0}"
FORMED_RADIUS="${FORMED_RADIUS:-250.0}"
CATCH_SPEED="${CATCH_SPEED:-20.0}"
CATCH_RADIUS="${CATCH_RADIUS:-400.0}"

# ── SITL --defaults 疊層 ────────────────────────────────────────────
DEF_LIST=""
for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
         "$BR/sitl_extra.parm" \
         "$BASE/sitl_overrides.parm"; do
  [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
done
if [ "$LT" = "catapult" ] && [ -f "$BR/sitl_catapult.parm" ]; then
  DEF_LIST="${DEF_LIST:+$DEF_LIST,}$BR/sitl_catapult.parm"
fi

echo "[formation] LOC=$LOC_RAW → $LOC | N=$N | FORMATION=${FORMATION:-v}(bio=$BIO) | LAUNCH=$LT"
echo "[formation] auto: arm=$AA takeoff=$AT patrol=$AP_PATROL raft=$RAFT_ARG | origin=$BLAT,$BLON hdg=$BHDG"

if [ ! -x "$AP" ]; then
  echo "[formation] ERROR: 找不到 arduplane binary: $AP" >&2
  echo "[formation]  build：cd ~/ardupilot && ./waf configure --board sitl && ./waf plane" >&2
  exit 1
fi
for s in "$ROS_SETUP" "$WS_SETUP"; do
  [ -f "$s" ] || { echo "[formation] ERROR: 找不到 ROS setup: $s" >&2; exit 1; }
done

# ── 清理殘留 ────────────────────────────────────────────────────────
echo "[formation] cleanup 殘留 SITL / MAVProxy / MAVROS / nodes"
pkill -x arduplane 2>/dev/null || true
pkill -f sim_vehicle 2>/dev/null || true
pkill -f mavproxy.py 2>/dev/null || true
pkill -x mavros_node 2>/dev/null || true
for nd in master_node wingman_node backup_node; do pkill -f "$nd" 2>/dev/null || true; done
sleep 3

# ── 1) 6×SITL（單縱列沿跑道排開）──────────────────────────────────
echo "[formation] 1) 啟動 $N 台 arduplane（model=$MODEL，單縱列）"
for i in $(seq 0 $((N - 1))); do
  n=$((i + 1))
  wd="$BASE/uav$n"; mkdir -p "$wd"; rm -f "$wd/eeprom.bin" "$wd/mav.parm"
  # 僚機排在長機正後方 i*SPACING 公尺（沿 BHDG 反向），避免地面重疊
  home=$(awk -v lat="$BLAT" -v lon="$BLON" -v alt="$BALT" -v hdg="$BHDG" -v i="$i" -v s="$SPACING" \
    'BEGIN{ pi=3.141592653589793; br=(hdg+180.0)*pi/180.0; d=i*s;
            dlat=(d*cos(br))/111320.0; dlon=(d*sin(br))/(111320.0*cos(lat*pi/180.0));
            printf "%.7f,%.7f,%s,%s", lat+dlat, lon+dlon, alt, hdg }')
  ( cd "$wd" && exec "$AP" -w --model "$MODEL" --home "$home" \
       -I"$i" --sysid "$n" ${DEF_LIST:+--defaults "$DEF_LIST"} ) \
       > "/tmp/sitl_uav$n.log" 2>&1 &
done
up=0
for _t in $(seq 1 60); do
  up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((5760 + 10 * i)) " && up=$((up + 1)); done
  [ "$up" -ge "$N" ] && break; sleep 1
done
echo "[formation]    SITL ports up=$up/$N"

# ── 2) MAVProxy fan-out（每台一個；MAVROS 5860 / AeroPlan 6760 / node 6960）──
# SITL 單一 TCP client 限制 → MAVProxy 當唯一 client，再 tcpin 分流給三方消費者。
# ⚠️ headless（無 TTY）下 mavproxy --non-interactive 會因 stdin EOF 立即退出
#    （--daemon 在本機不可靠）。用 `tail -f /dev/null |` 餵永不 EOF 的 stdin，
#    再 setsid 脫離 session → 常駐。這是本腳本不呼叫 bridge_mavproxy_swarm.sh
#    而自行 fan-out 的原因（後者靠 --daemon，被 script 化時會被 stdin EOF 打死）。
# ⚠️ 半數 fan-out 起不來的成因：前一次 mavproxy 沒清乾淨仍佔埠 → 新實例 bind
#    撞 [Errno 98] Address already in use 而死。對策：逐台驗證 AeroPlan 埠、重啟自癒。
echo "[formation] 2) MAVProxy fan-out（MAVROS 5860 / AeroPlan 6760 / node 6960）"
mkdir -p /tmp/mavbridge
pkill -f mavproxy.py 2>/dev/null || true
pkill -f "[t]ail -f /dev/null" 2>/dev/null || true
sleep 2
_launch_bridge() {   # $1 = instance index i
  local i="$1" n=$(( $1 + 1 ))
  local m=$((5760 + 10 * i)) mp=$((5860 + 10 * i)) ap=$((6760 + 10 * i)) np=$((6960 + 10 * i))
  setsid bash -c "tail -f /dev/null | mavproxy.py \
      --master=tcp:127.0.0.1:$m \
      --out=tcpin:0.0.0.0:$mp --out=tcpin:0.0.0.0:$ap --out=tcpin:0.0.0.0:$np \
      --source-system=255 --non-interactive --state-basedir=/tmp/mavbridge \
      --logfile=/tmp/mavbridge/uav$n.tlog" >/tmp/mavbridge/uav$n.log 2>&1 &
}
_kill_bridge() { for pid in $(pgrep -f "master=tcp:127.0.0.1:$((5760 + 10 * $1))" 2>/dev/null); do kill -9 "$pid" 2>/dev/null; done; }
for i in $(seq 0 $((N - 1))); do _launch_bridge "$i"; done
for attempt in 1 2 3 4; do
  sleep 3
  miss=""
  for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " || miss="$miss $i"; done
  [ -z "$miss" ] && break
  echo "[formation]    retry #$attempt：UAV$(for i in $miss; do printf '%s ' $((i+1)); done)分流未起，重啟"
  for i in $miss; do _kill_bridge "$i"; sleep 1; _launch_bridge "$i"; done
done
up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " && up=$((up + 1)); done
echo "[formation]    MAVProxy fan-out ports up=$up/$N (AeroPlan 6760+10i)"

# ── 3) 6×MAVROS（連 bridge 的 5860+10i）────────────────────────────
echo "[formation] 3) 啟動 $N 台 MAVROS（FCU_TCP_BASE=5860）"
# shellcheck disable=SC1090
source "$ROS_SETUP"; source "$WS_SETUP"
FCU_TCP_BASE=5860 COUNT="$N" bash "$BR/start_mavros_swarm.sh" > /tmp/mavros_all.log 2>&1 &
# 等長機 MAVROS 連上（最多 ~40s）再拉編隊節點，避免節點空轉
echo "[formation]    等待 /uav1/mavros/state connected..."
for _t in $(seq 1 40); do
  st=$(timeout 2 ros2 topic echo --once /uav1/mavros/state 2>/dev/null | grep -c "connected: true" || true)
  [ "${st:-0}" -ge 1 ] && { echo "[formation]    MAVROS UAV1 connected"; break; }
  sleep 1
done

# ── 4) sar_swarm 編隊節點 ──────────────────────────────────────────
# origin_lat/lon/heading_deg 傳入 → 節點的巡邏方框與生成點同址（修「編隊起點≠SITL
# 導致順移」）。
echo "[formation] 4) sar_swarm 編隊節點（use_biomimetic=$BIO, origin=$BLAT,$BLON）"
ros2 launch sar_swarm sar_failover_nodes.launch.py \
    use_mavros:=true use_biomimetic:="$BIO" launch_type:="$LT" \
    auto_arm:="$AA" auto_takeoff:="$AT" auto_patrol:="$AP_PATROL" \
    with_raft:="$RAFT_ARG" \
    origin_lat:="$BLAT" origin_lon:="$BLON" heading_deg:="$BHDG" \
    nominal_speed:="$NOMINAL_SPEED" wait_speed:="$WAIT_SPEED" \
    formed_radius:="$FORMED_RADIUS" catch_speed:="$CATCH_SPEED" \
    catch_radius:="$CATCH_RADIUS" \
    > /tmp/nodes.log 2>&1 &
sleep 3

WSL_IP="$(hostname -I | awk '{print $1}')"
echo "[formation] ═══════════════════════════════════════════════════"
echo "[formation] 全部啟動完成。WSL_IP=$WSL_IP"
echo "[formation] AeroPlan 連線：SITL 分頁 → 嵌入式蜂群 → host=$WSL_IP base_port=6760 stride=10 count=$N sysid_base=1"
echo "[formation] 監看話題：ros2 topic echo /swarm/heartbeat"
echo "[formation] 故障切換示範：pkill -f master_node（備援 UAV6 ~2.5s 接手）"
echo "[formation] logs：/tmp/sitl_uav<N>.log /tmp/bridge.log /tmp/mavros_uav<N>.log /tmp/nodes.log"
echo "[formation] ═══════════════════════════════════════════════════"
echo "[formation] wait 常駐（Ctrl+C 或 swarm_stop.sh 停止）"
wait
