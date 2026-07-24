#!/usr/bin/env bash
# swarm_ros2_up.sh — 第 2 層：6×MAVROS + 長僚機（master/wingman/backup）ROS2 launch。
#
# 與 swarm_sitl_up.sh 成對使用（SITL / ROS2 控制分離）：
#   前置：swarm_sitl_up.sh 已在跑（分流埠 5860+10i 已 LISTEN）。
#   本層可隨時 swarm_ros2_down.sh 停掉再重開 —— SITL 不受影響。
#
# origin / launch_type / 台數 自動讀 ~/fixwing_ws/sitl_run/swarm_state.env
# （swarm_sitl_up.sh 寫入），保證編隊原點 = SITL 生成點（不會順移）。
# 也可用 env 顯式覆寫（LOC= / LAUNCH= / COUNT=）。
#
# 用法：
#   bash swarm_ros2_up.sh                       # 待命：不 arm 不起飛（預設 MANUAL=1）
#   MANUAL=0 bash swarm_ros2_up.sh              # 全自動 arm+起飛+巡邏
#   PATROL=0 bash swarm_ros2_up.sh              # 你起飛後長機定點（不巡邏），僚機仍編隊
#   FORMATION=biomimetic bash swarm_ros2_up.sh  # 仿生 V
#   WAIT_SPEED=6 FORMED_RADIUS=120 CATCH_SPEED=25 ...   # 編隊位置約束調校
#
# 之後起飛由你觸發：bash swarm_takeoff.sh [n...]
# 停止本層：bash swarm_ros2_down.sh（或 Ctrl+C）
set -eo pipefail

BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
STATE="$BASE/swarm_state.env"

# ── 讀 SITL 層狀態檔（可被 env 覆寫）──────────────────────────────
SWARM_LOC=""; SWARM_LAUNCH=""; SWARM_N=""
# shellcheck disable=SC1090
[ -f "$STATE" ] && source "$STATE"
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    TAOYUAN|RCTP|TPE) echo "25.0777,121.2325,32,52" ;;
    HUWEI)      echo "23.702732,120.419333,0,0"  ;;
    SWARM)      echo "22.60,120.00,0,90" ;;
    "")         echo "" ;;
    *)          echo "$1" ;;
  esac
}
LOC="$(resolve_loc "${LOC:-}")"
LOC="${LOC:-${SWARM_LOC:-25.0777,121.2325,32,52}}"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"
LT="${LAUNCH:-${SWARM_LAUNCH:-runway}}"
N="${COUNT:-${SWARM_N:-6}}"

case "$(echo "${FORMATION:-v}" | tr '[:upper:]' '[:lower:]')" in
  biomimetic|bio|swarm) BIO=true ;;
  *)                    BIO=false ;;
esac

# MANUAL=1（預設）：節點不 arm 不起飛，起飛由 swarm_takeoff.sh / GCS 觸發；
# 爬過 15m 後節點自動接手 GUIDED 編隊。PATROL 獨立（預設 1 = 起飛後長機帶隊巡邏）。
MANUAL="${MANUAL:-1}"
if [ "$MANUAL" = "1" ]; then AA=false; AT=false; else AA=true; AT=true; fi
PATROL="${PATROL:-1}"
[ "$PATROL" = "1" ] && AP_PATROL=true || AP_PATROL=false
RAFT_ARG="${RAFT:-0}"; [ "$RAFT_ARG" = "1" ] && RAFT_ARG=true || RAFT_ARG=false

# ══ PRESET：一鍵套用「收斂 vs 速度」兩組實測調參 ═══════════════════════════
# 實測結論（收斂與速度是物理取捨，非調參 bug）：
#   固定翼轉彎半徑 r≈v²/(g·tanφ)。要把僚機「收」進 slot，catch 速度的轉彎半徑
#   必須 < 編隊間隔；速度越高半徑越大 → 高速下僚機只能繞著 slot 打轉收不進來。
#   另外長機要能「慢下來等」才有速度差讓僚機追上。
#     • tight（泡棉 plane 模型，min~10 / max~30 m/s）：長機可慢到 12、僚機追到 30，
#       速度差大、轉彎半徑小 → 實測 worst slot err 收斂到 ~50m（直線）~170m（轉角），
#       全程編隊成形、長機會自動 12↔22 手風琴等待。但巡航僅 ~22 m/s。
#     • fast（噴射 plane-jet，實測最低可持平飛 ~45 / max~76 m/s）：速度快，但最低
#       平飛速度就給出 ~500m 轉彎半徑（比任何合理間隔都大），物理上收不緊 →
#       實測只能維持 ~300m 內的凝聚雲團（不脫隊、不亂飛，但非緊 V）。
#   ⇒ 想「緊 V」用 tight（配 SITL：不要 JET）；想「最快」用 fast（配 SITL：JET=1）。
# 用法：PRESET=tight bash swarm_ros2_up.sh  /  PRESET=fast bash swarm_ros2_up.sh
case "$(echo "${PRESET:-}" | tr '[:upper:]' '[:lower:]')" in
  tight|foam|slow)
    : "${NOMINAL_SPEED:=22.0}" "${WAIT_SPEED:=12.0}" "${CATCH_SPEED:=30.0}"
    : "${CATCH_RADIUS:=500.0}" "${FORMED_RADIUS:=50.0}"
    : "${V_SPACING:=60.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=120.0}"
    : "${PATROL_BOX:=1500.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=6.0}"
    echo "[ros2] PRESET=tight（緊 V，需 SITL 用泡棉 plane 模型／不要 JET=1）" ;;
  fast|jet)
    : "${NOMINAL_SPEED:=35.0}" "${WAIT_SPEED:=45.0}" "${CATCH_SPEED:=76.0}"
    : "${CATCH_RADIUS:=900.0}" "${FORMED_RADIUS:=150.0}"
    : "${V_SPACING:=120.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=220.0}"
    : "${PATROL_BOX:=4000.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=10.0}"
    echo "[ros2] PRESET=fast（最快 ~76 m/s，凝聚雲團非緊 V；需 SITL JET=1）" ;;
  max|plana|maxv)
    # Plan A 實測冠軍（2026-07-23 iter7）：泡棉機頂到氣動天花板還保住 V。
    # 三個關鍵機制（缺一不可，前兩個是程式修改）：
    #   1. speed_for_error 斜坡從 err=0 起算（否則 err 漸近卡在 formed_r 上緣永不釋放）
    #   2. wp_lead_time slot 前饋（WP 瞄外推點抵消 ~3.5s 導引滯後；誤差仍對真 slot 量）
    #   3. formed_hysteresis 1.8（巡航誤差膨脹 ~+70m，1.4 會一釋放就被打回 wait）
    # 實測：長機平均 25.0 m/s、巡航佔比 85%、誤差 95(直線)~173(轉角) 有界手風琴、
    # 零脫隊。需 SITL 泡棉 + MAXV=1（AIRSPEED_MAX 35 解 30 參數上限；氣動極限 ~31.5）。
    : "${NOMINAL_SPEED:=27.0}" "${WAIT_SPEED:=16.0}" "${CATCH_SPEED:=31.0}"
    : "${CATCH_RADIUS:=300.0}" "${FORMED_RADIUS:=95.0}" "${FORMED_HYST:=1.8}"
    : "${WP_LEAD:=3.5}"
    : "${V_SPACING:=60.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=120.0}"
    : "${PATROL_BOX:=2500.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=6.0}"
    echo "[ros2] PRESET=max（Plan A：巡航 ~25-27 m/s + V 隊形；SITL 用泡棉 + MAXV=1）" ;;
esac

# 編隊位置約束（長機等僚機 / 僚機追上）調校旋鈕
NOMINAL_SPEED="${NOMINAL_SPEED:-14.0}"
WAIT_SPEED="${WAIT_SPEED:-10.0}"
FORMED_RADIUS="${FORMED_RADIUS:-250.0}"
CATCH_SPEED="${CATCH_SPEED:-20.0}"
CATCH_RADIUS="${CATCH_RADIUS:-400.0}"
# V 隊形幾何（間隔）：V_SPACING=編隊間隔 m（越小越密）；V_ANGLE=V 半角；V_TAIL=備長跟隨距離
# ⚠️ 要收緊到 ~50m：V_SPACING=50 且 FORMED_RADIUS 要 < 50（例 30），否則永遠算「到位」不收緊。
V_SPACING="${V_SPACING:-120.0}"
V_ANGLE="${V_ANGLE:-25.0}"
V_TAIL="${V_TAIL:-220.0}"
# WP 追蹤：導引航點刷新間隔 / slot 移動門檻。越小＝slot 航點越連續 → 少 S 形蛇行。
# 預設已收緊到 1.0/10（節點內建 3.0/25 對移動編隊太粗會蛇行）。
WP_INTERVAL="${WP_INTERVAL:-1.0}"
WP_MOVE_THRESH="${WP_MOVE_THRESH:-10.0}"
# 長機/編隊巡航高度帶（origin altitude 參數）。⚠️ 不可用舊 sim 值 1000，否則長機
# 會爬到 1000m 遠離編隊(patrol_alt≈80m)看似不收斂。預設 100 與編隊同高度帶。
ORIGIN_ALT="${ORIGIN_ALT:-100.0}"
# 巡邏方框邊長 m。高速下加大（直線段長、轉角少）→ 僚機有足夠直線收斂 slot。
PATROL_BOX="${PATROL_BOX:-1500.0}"
# 成形解除門檻倍數（master formed_hysteresis）。巡航中 slot 誤差比等待時膨脹 ~+70m,
# 1.4 太窄會「一釋放就被打回 wait」群速卡在 wait_speed;拉高 nominal 時建議 ~1.8。
FORMED_HYST="${FORMED_HYST:-1.4}"
# slot 前饋秒數:目標 = slot 沿長機航向外推 lead×leader_speed,抵消導引 WP ~3-5s
# 有效滯後(高速巡航誤差地板 ≈ v×lag,27m/s 時 ~100-150m)。0=關;高速編隊建議 3-4。
WP_LEAD="${WP_LEAD:-0.0}"
# §4.5 轉彎程序:弧轉半徑(slot 掃掠率=wait_speed/R;大=轉角僚機跟得上)+ 轉前減速距離
TURN_RADIUS="${TURN_RADIUS:-800.0}"
PRESLOW_MARGIN="${PRESLOW_MARGIN:-250.0}"

echo "[ros2] origin=$BLAT,$BLON hdg=$BHDG | N=$N | LAUNCH=$LT | bio=$BIO"
echo "[ros2] manual=$MANUAL patrol=$AP_PATROL raft=$RAFT_ARG | wait=$WAIT_SPEED formed_r=$FORMED_RADIUS catch=$CATCH_SPEED"

# ── 前置檢查：SITL 層分流埠要在 ────────────────────────────────────
if ! ss -ltn 2>/dev/null | grep -q ":5860 "; then
  echo "[ros2] ERROR: 分流埠 5860 沒在 LISTEN —— 先跑 bash swarm_sitl_up.sh" >&2
  exit 1
fi

# ── 清理殘留（只清本層：mavros + 節點；不動 SITL/mavproxy）──────────
pkill -x mavros_node 2>/dev/null || true
for p in "[m]aster_node" "[w]ingman_node" "[b]ackup_node" "[a]pm_sitl"; do
  pkill -f "$p" 2>/dev/null || true
done
sleep 2
trap 'echo; echo "[ros2] 停止 ROS2 層（SITL 不動）..."; pkill -x mavros_node 2>/dev/null; for p in "[m]aster_node" "[w]ingman_node" "[b]ackup_node" "[a]pm_sitl"; do pkill -f "$p" 2>/dev/null; done; exit 0' INT TERM

# ── 1) 6×MAVROS（連分流 5860+10i）─────────────────────────────────
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash"
echo "[ros2] 1) 啟動 $N 台 MAVROS（FCU_TCP_BASE=5860）"
FCU_TCP_BASE=5860 COUNT="$N" bash "$BR/start_mavros_swarm.sh" > /tmp/mavros_all.log 2>&1 &
echo "[ros2]    等待 /uav1/mavros/state connected..."
for _t in $(seq 1 40); do
  st=$(timeout 2 ros2 topic echo --once /uav1/mavros/state 2>/dev/null | grep -c "connected: true" || true)
  [ "${st:-0}" -ge 1 ] && { echo "[ros2]    MAVROS UAV1 connected"; break; }
  sleep 1
done

# ── 2) 長僚機節點（master=UAV1 / wingman=2-5 / backup=UAV6）────────
echo "[ros2] 2) sar_swarm 長僚機節點"
ros2 launch sar_swarm sar_failover_nodes.launch.py \
    use_mavros:=true use_biomimetic:="$BIO" launch_type:="$LT" \
    auto_arm:="$AA" auto_takeoff:="$AT" auto_patrol:="$AP_PATROL" \
    with_raft:="$RAFT_ARG" \
    origin_lat:="$BLAT" origin_lon:="$BLON" heading_deg:="$BHDG" altitude:="$ORIGIN_ALT" \
    nominal_speed:="$NOMINAL_SPEED" wait_speed:="$WAIT_SPEED" \
    formed_radius:="$FORMED_RADIUS" catch_speed:="$CATCH_SPEED" \
    catch_radius:="$CATCH_RADIUS" \
    v_spacing_m:="$V_SPACING" v_angle_deg:="$V_ANGLE" v_tail_dist_m:="$V_TAIL" \
    wp_interval:="$WP_INTERVAL" wp_move_thresh:="$WP_MOVE_THRESH" \
    patrol_box_m:="$PATROL_BOX" formed_hysteresis:="$FORMED_HYST" \
    wp_lead_time:="$WP_LEAD" turn_radius:="$TURN_RADIUS" preslow_margin:="$PRESLOW_MARGIN" \
    > /tmp/nodes.log 2>&1 &
sleep 3

echo "[ros2] ═══════════════════════════════════════════════════"
echo "[ros2] ROS2 層就緒（常駐）。"
if [ "$MANUAL" = "1" ]; then
  echo "[ros2] 待命模式：飛機在地面 disarmed，起飛由你觸發："
  echo "[ros2]     bash swarm_takeoff.sh          # 全部（間隔 STAGGER=5s）"
  echo "[ros2]     bash swarm_takeoff.sh 1        # 只放長機"
fi
echo "[ros2] 監看：ros2 topic echo /swarm/heartbeat   節點 log：tail -f /tmp/nodes.log"
echo "[ros2] 停止本層（SITL 不動）：bash swarm_ros2_down.sh"
echo "[ros2] ═══════════════════════════════════════════════════"
wait
