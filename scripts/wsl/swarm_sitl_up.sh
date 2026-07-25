#!/usr/bin/env bash
# swarm_sitl_up.sh — 第 1 層：6 台 SITL + MAVProxy 分流，常駐（不含任何 ROS2）。
#
# 與 swarm_ros2_up.sh 成對使用（SITL / ROS2 控制分離）：
#   終端 A：bash swarm_sitl_up.sh          # SITL 層常駐（本腳本）
#   終端 B：bash swarm_ros2_up.sh          # ROS2 層（6×MAVROS + 長僚機節點）
# ROS2 層可隨時 swarm_ros2_down.sh 停掉重開，SITL 完全不受影響（飛機還在飛）。
#
# 分流埠（每台 i=0..N-1）：
#   SITL master  tcp 5760+10i（MAVProxy 是唯一 client）
#   MAVROS       tcpin 5860+10i
#   AeroPlan     tcpin 6760+10i（嵌入式蜂群 base_port 6760）
#   節點 WP 鏈路 tcpin 6960+10i
#
# 用法：
#   bash swarm_sitl_up.sh                    # 桃園機場、6 台、跑道起飛
#   LOC=HUWEI bash swarm_sitl_up.sh          # 虎尾
#   LOC=23.5,120.3,0,90 N=4 SPACING=50 ...   # 自訂
#   LAUNCH=catapult bash swarm_sitl_up.sh    # 彈射（frame plane-catapult）
#
# 生成參數會寫入 ~/fixwing_ws/sitl_run/swarm_state.env，
# swarm_ros2_up.sh 讀它對齊 origin / launch_type（避免編隊原點 ≠ 生成點的順移 bug）。
#
# 停止：bash swarm_sitl_down.sh（或 Ctrl+C；會一併收掉 mavproxy）
# ⚠️ 結尾 wait 常駐 —— SITL 設 PR_SET_PDEATHSIG，本腳本就是它們的命；別關這個終端。
set -eo pipefail

AP="$HOME/ardupilot/build/sitl/bin/arduplane"
BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
STATE="$BASE/swarm_state.env"
export PATH="$HOME/.local/bin:$PATH"

# ── 出生地點預設（與 AeroPlan config/spawn_presets.py 同址）────────────
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    TAOYUAN|RCTP|TPE|"") echo "25.0777,121.2325,32,52" ;;  # 桃園機場 跑道05（預設）
    HUWEI)      echo "23.702732,120.419333,0,0"  ;;        # 虎尾科大
    SWARM)      echo "22.60,120.00,0,90" ;;                # 舊蜂群原點
    *)          echo "$1" ;;                               # 已是 lat,lon,alt,hdg
  esac
}
LOC_RAW="${LOC:-TAOYUAN}"
LOC="$(resolve_loc "$LOC_RAW")"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"

N="${N:-6}"
SPACING="${SPACING:-30}"          # 縱列間距 [m]

case "$(echo "${LAUNCH:-runway}" | tr '[:upper:]' '[:lower:]')" in
  catapult) MODEL="plane-catapult"; LT="catapult" ;;
  *)        MODEL="plane";          LT="runway"   ;;
esac

# ── JET=1：高速噴射物理模型（plane-jet，巡航 40 / 最大 ~80 m/s）──────
# 泡棉機 plane 上限 ~30 m/s；要更快必須換 plane-jet 物理模型。這裡並把
# AIRSPEED_MAX / ARSPD_FBW_MAX 解到很高（AIRSPEED_MAX_CAP，預設 120），讓它
# 飛到模型物理極限而非被參數卡住。實際上限由 plane-jet 氣動/推力決定（~80-100）。
JET_BASE="$HOME/ardupilot/Tools/autotest/default_params/plane-jet.parm"
if [ "${JET:-0}" = "1" ]; then
  # SIM_Plane 用獨立的 strstr 解析 frame 字串（-jet / -catapult 各自成立），
  # 所以噴射＋彈射要合成 "plane-jet-catapult"。
  # ⚠️ 舊版此處直接 MODEL="plane-jet"，會把 LAUNCH=catapult 選到的模型蓋掉 →
  #    彈射器 silently 消失（have_launcher=false），只能跑道滾行起飛。
  if [ "$LT" = "catapult" ]; then MODEL="plane-jet-catapult"; else MODEL="plane-jet"; fi
  CAP="${AIRSPEED_MAX_CAP:-150}"
  # 姿態限制解除（噴射機不該被泡棉級的角度上限綁住）。DIVE=0 可關掉這段。
  # ⚠️ 用 .parm 在 SITL 啟動時載入，而非事後 param_set —— 分流讀回參數不可靠，
  #    「送出去了但沒吃進去」曾讓終端俯衝只掉 10 m/s（該有 ~71 m/s）而高空掠過。
  DIVE_PITCH_MIN="${DIVE_PITCH_MIN:--80}"   # 最大低頭角（負值；-80 = 幾乎垂直俯衝）
  DIVE_PITCH_MAX="${DIVE_PITCH_MAX:-45}"
  DIVE_ROLL_MAX="${DIVE_ROLL_MAX:-80}"
  DIVE_SINK_MAX="${DIVE_SINK_MAX:-120}"     # 45° @100 m/s 需 ~71 m/s 下沉率
  cat > "$BASE/jet_fast.parm" <<EOF
# 由 swarm_sitl_up.sh JET=1 產生 — 解除空速上限，飛到模型物理極限
AIRSPEED_MAX $CAP
ARSPD_FBW_MAX $CAP
ARSPD_USE 1
EOF
  if [ "${DIVE:-1}" = "1" ]; then
    cat >> "$BASE/jet_fast.parm" <<EOF
# ── 終端俯衝：解除角度/下沉限制（參數名已對照本 build 原始碼確認）──
PTCH_LIM_MIN_DEG $DIVE_PITCH_MIN
PTCH_LIM_MAX_DEG $DIVE_PITCH_MAX
ROLL_LIMIT_DEG $DIVE_ROLL_MAX
STALL_PREVENTION 0
TECS_SINK_MAX $DIVE_SINK_MAX
TECS_SINK_MIN 5
TECS_PITCH_MIN $DIVE_PITCH_MIN
TECS_PITCH_MAX $DIVE_PITCH_MAX
TECS_SPDWEIGHT 1
THR_MIN 0
EOF
  fi
fi

# ── MAXV=1（泡棉專用）：解除 30 m/s 參數上限，配 ROS2 層 PRESET=max ─────────
# 泡棉 plane 的 30 是「參數」上限（models/plane.parm AIRSPEED_MAX 30）;氣動天花板
# 其實 ~31.5（SIM_Plane: 推力 28N = 阻力 @31.5-32）。解到 35 讓 TECS 用滿氣動極限。
# ⚠️ 此 build 參數已改名（post-4.4）：AIRSPEED_MAX/MIN、AIRSPEED_CRUISE（m/s）；
# 舊名 ARSPD_FBW_*/TRIM_ARSPD_CM 會被靜默忽略。
if [ "${MAXV:-0}" = "1" ] && [ "${JET:-0}" != "1" ]; then
  cat > "$BASE/sitl_overrides.parm" <<EOF
# 由 swarm_sitl_up.sh MAXV=1 產生 — 泡棉頂速（配 swarm_ros2_up.sh PRESET=max）
AIRSPEED_MAX 35
AIRSPEED_MIN 9
AIRSPEED_CRUISE 27
EOF
  echo "[sitl] MAXV=1：泡棉空速上限 30→35（氣動極限 ~31.5），cruise 27"
fi

# --defaults 疊層。JET=1 用 plane-jet.parm（噴射氣動/調參）取代 plane.parm，
# 再疊 jet_fast（解上限）；否則用 plane.parm（加速度校正）。
DEF_LIST=""
if [ "${JET:-0}" = "1" ]; then
  # ⚠️ models/plane.parm 必須先疊:它帶 12 行 INS_ACC* 加速度校正,而
  #    plane-jet.parm 一行都沒有。少了它 → 「PreArm: 3D Accel calibration needed」
  #    → 六機全部解不了鎖(彈射/起飛靜默失敗)。之後才疊 plane-jet 的噴射氣動調參。
  for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
           "$JET_BASE" "$BR/sitl_extra.parm" "$BASE/sitl_overrides.parm" "$BASE/jet_fast.parm"; do
    [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
  done
else
  for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
           "$BR/sitl_extra.parm" \
           "$BASE/sitl_overrides.parm"; do
    [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
  done
fi
[ "$LT" = "catapult" ] && [ -f "$BR/sitl_catapult.parm" ] && \
  DEF_LIST="${DEF_LIST:+$DEF_LIST,}$BR/sitl_catapult.parm"

echo "[sitl] LOC=$LOC_RAW → $LOC | N=$N | SPACING=${SPACING}m | LAUNCH=$LT"
[ -x "$AP" ] || { echo "[sitl] ERROR: 找不到 $AP" >&2; exit 1; }

# ── 清理殘留（只清本層：arduplane + mavproxy）──────────────────────
pkill -x arduplane 2>/dev/null || true
pkill -f "[m]avproxy.py" 2>/dev/null || true
pkill -f "[t]ail -f /dev/null" 2>/dev/null || true
sleep 2
trap 'echo; echo "[sitl] 停止 SITL 層..."; pkill -x arduplane 2>/dev/null; pkill -f "[m]avproxy.py" 2>/dev/null; pkill -f "[t]ail -f /dev/null" 2>/dev/null; exit 0' INT TERM

# ── 寫狀態檔（swarm_ros2_up.sh 讀取以對齊 origin / launch_type / N）──
cat > "$STATE" <<EOF
# 由 swarm_sitl_up.sh 自動產生 — swarm_ros2_up.sh 讀此對齊參數
SWARM_LOC="$LOC"
SWARM_LAUNCH="$LT"
SWARM_N="$N"
EOF
echo "[sitl] 狀態檔已寫入: $STATE"

# ── 6×SITL（單縱列，沿跑道航向排開）───────────────────────────────
echo "[sitl] 啟動 $N 台 arduplane（model=$MODEL，單縱列）"
for i in $(seq 0 $((N - 1))); do
  n=$((i + 1))
  wd="$BASE/uav$n"; mkdir -p "$wd"; rm -f "$wd/eeprom.bin" "$wd/mav.parm"
  home=$(awk -v lat="$BLAT" -v lon="$BLON" -v alt="$BALT" -v hdg="$BHDG" -v i="$i" -v s="$SPACING" \
    'BEGIN{ pi=3.141592653589793; br=(hdg+180.0)*pi/180.0; d=i*s;
            dlat=(d*cos(br))/111320.0; dlon=(d*sin(br))/(111320.0*cos(lat*pi/180.0));
            printf "%.7f,%.7f,%s,%s", lat+dlat, lon+dlon, alt, hdg }')
  printf '  UAV%-2d -I%-2d tcp:%-5d @ %s\n' "$n" "$i" "$((5760 + 10 * i))" "$home"
  ( cd "$wd" && exec "$AP" -w --model "$MODEL" --home "$home" \
       -I"$i" --sysid "$n" ${DEF_LIST:+--defaults "$DEF_LIST"} ) \
       > "/tmp/sitl_uav$n.log" 2>&1 &
done
up=0
for _t in $(seq 1 60); do
  up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((5760 + 10 * i)) " && up=$((up + 1)); done
  [ "$up" -ge "$N" ] && break; sleep 1
done
echo "[sitl] SITL ports up=$up/$N"

# ── MAVProxy 分流（stdin keepalive + 逐台驗證/重啟自癒）──────────────
# 半數 fan-out 起不來的成因：前一次 bring-up 的 mavproxy 沒清乾淨仍佔埠 →
# 新實例 bind 撞 [Errno 98] Address already in use 而死 → 只剩前幾台（AeroPlan
# 就會「有些連得到、有些連不到」）。對策：先等舊埠釋放，再啟動；啟動後以
# AeroPlan 埠 6760+10i 為準逐台驗證，沒起來的 kill 掉重啟，重試數輪。
echo "[sitl] MAVProxy 分流（MAVROS 5860+ / AeroPlan 6760+ / 節點 6960+）"
mkdir -p /tmp/mavbridge

_launch_bridge() {   # $1 = instance index i
  local i="$1" n=$(( $1 + 1 ))
  local m=$((5760 + 10 * i)) mp=$((5860 + 10 * i)) ap=$((6760 + 10 * i)) np=$((6960 + 10 * i))
  setsid bash -c "tail -f /dev/null | mavproxy.py \
      --master=tcp:127.0.0.1:$m \
      --out=tcpin:0.0.0.0:$mp --out=tcpin:0.0.0.0:$ap --out=tcpin:0.0.0.0:$np \
      --source-system=255 --non-interactive --state-basedir=/tmp/mavbridge \
      --logfile=/tmp/mavbridge/uav$n.tlog" >/tmp/mavbridge/uav$n.log 2>&1 &
}
_kill_bridge() {     # $1 = instance index i；殺該實例的 mavproxy（依 master 埠精準比對）
  local m=$((5760 + 10 * $1))
  for pid in $(pgrep -f "master=tcp:127.0.0.1:$m" 2>/dev/null); do kill -9 "$pid" 2>/dev/null; done
}

# 先確保目標 fan-out 埠都空著（等舊 mavproxy 釋放，最多 ~8s）
for _t in $(seq 1 8); do
  busy=0
  for i in $(seq 0 $((N - 1))); do
    ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " && busy=$((busy + 1))
  done
  [ "$busy" -eq 0 ] && break
  _kill_bridge_all() { for i in $(seq 0 $((N-1))); do _kill_bridge "$i"; done; }
  _kill_bridge_all; sleep 1
done

# 首發全部起
for i in $(seq 0 $((N - 1))); do _launch_bridge "$i"; done
# 驗證 + 重啟自癒（以 AeroPlan 埠為準；AeroPlan 實際連的就是這個）
for attempt in 1 2 3 4; do
  sleep 3
  miss=""
  for i in $(seq 0 $((N - 1))); do
    ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " || miss="$miss $i"
  done
  [ -z "$miss" ] && break
  echo "[sitl]   retry #$attempt：UAV$(for i in $miss; do printf '%s ' $((i+1)); done)分流未起，重啟"
  for i in $miss; do _kill_bridge "$i"; sleep 1; _launch_bridge "$i"; done
done
up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " && up=$((up + 1)); done
echo "[sitl] AeroPlan 分流 ports up=$up/$N（6760..$((6760 + (N - 1) * 10))）"
[ "$up" -lt "$N" ] && echo "[sitl] ⚠ 仍有 $((N - up)) 台未起 —— 檢查 /tmp/mavbridge/uav<N>.log"

WSL_IP="$(hostname -I | awk '{print $1}')"
echo "[sitl] ═══════════════════════════════════════════════════"
echo "[sitl] SITL 層就緒（常駐）。WSL_IP=$WSL_IP"
echo "[sitl] AeroPlan：嵌入式蜂群 → host=$WSL_IP base_port=6760 stride=10 count=$N"
echo "[sitl] 下一步：另開終端跑 bash swarm_ros2_up.sh（讀 $STATE 自動對齊 origin）"
echo "[sitl] 停止本層：bash swarm_sitl_down.sh"
echo "[sitl] ═══════════════════════════════════════════════════"
wait
