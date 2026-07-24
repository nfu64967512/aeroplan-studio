#!/usr/bin/env bash
# swarm_column_up.sh — 一鍵開 6 台固定翼 SITL，排成「一縱列」(single-file column)。
#
# 用途：把 WSL 內 fixwing_ws 的六台長航時固定翼 SITL 一次拉起來，生成點沿機頭
#       航向排成單縱列（一台接一台，nose-to-tail），適合順序起飛 / 隊形演練。
#
# 為什麼直接跑 arduplane binary（不用 sim_vehicle.py）：
#   本 WSL 的 sim_vehicle 會把 arduplane 包進 xterm，xterm 載不到字型
#   (cannot load font 10x20) → SITL 當場死掉。直接跑 binary 繞過此問題。
#   （見專案記憶 wsl-sitl-swarm-bridge 的 gotcha。）
#
# 用法：
#   bash swarm_column_up.sh                 # 預設 虎尾(HUWEI)、6 台、間距 30 m
#   LOC=TAOYUAN bash swarm_column_up.sh     # 改生成在 桃園機場 RCTP 跑道 05
#   LOC=23.5,120.3,0,90 bash ...            # 自訂 lat,lon,alt,heading
#   N=4 SPACING=50 bash ...                 # 4 台、間距 50 m
#   BRIDGE=1 bash ...                       # 額外開 MAVProxy fan-out（AeroPlan 用 6760）
#   COL_BEARING=180 bash ...                # 強制縱列朝正南（不跟航向）
#
# 以 run_in_background 或獨立終端執行 —— 結尾 wait 常駐，保住 SITL 父程序
# (ArduPilot SITL 設 PR_SET_PDEATHSIG，父程序退出就一起死)。
#
# 停止：bash swarm_stop.sh   或   pkill -x arduplane
#
# 注意：不可用 `set -u`（若 BRIDGE=1 會 source ROS setup.bash，碰未綁定變數會中止）。
set -eo pipefail

# ── 路徑 ────────────────────────────────────────────────────────────
AP="$HOME/ardupilot/build/sitl/bin/arduplane"
BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
# --defaults 疊層：models/plane.parm(3D 加速度校正，免 prearm 失敗)
#              + sitl_extra.parm(電池等) + sitl_overrides.parm(關 failsafe/arming check)
DEF_LIST=""
for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
         "$BR/sitl_extra.parm" \
         "$BASE/sitl_overrides.parm"; do
  [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
done

export PATH="$HOME/.local/bin:$PATH"

# ── 出生地點預設（與 AeroPlan Studio config/spawn_presets.py 同址）─────
# lat,lon,alt(m),heading(deg)
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    HUWEI|"")   echo "23.702732,120.419333,0,0"  ;;   # 虎尾科大 / 專案預設
    TAOYUAN|RCTP|TPE) echo "25.0777,121.2325,32,52" ;; # 桃園機場 跑道 05
    *)          echo "$1" ;;                           # 已是 lat,lon,alt,hdg
  esac
}
LOC_RAW="${LOC:-HUWEI}"
LOC="$(resolve_loc "$LOC_RAW")"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"

N="${N:-6}"                       # 台數
SPACING="${SPACING:-30}"          # 縱列間距 [m]（一台接一台）
# 縱列方向：預設 = 機頭航向的反向（僚機排在長機正後方，nose-to-tail）。
# 想要固定朝向（例如正南 180 = 地圖上垂直縱線）可用 COL_BEARING 覆寫。
COL_BEARING="${COL_BEARING:-}"
BRIDGE="${BRIDGE:-0}"             # 1 → 同時開 MAVProxy fan-out（AeroPlan 6760）

echo "[column] LOC=$LOC_RAW → $LOC | N=$N | SPACING=${SPACING}m | bearing=${COL_BEARING:-hdg+180}"
echo "[column] defaults: ${DEF_LIST:-<none>}"

if [ ! -x "$AP" ]; then
  echo "[column] ERROR: 找不到 arduplane binary: $AP" >&2
  echo "[column] 請先 build ArduPlane SITL：cd ~/ardupilot && ./waf configure --board sitl && ./waf plane" >&2
  exit 1
fi

# ── 清理殘留 ────────────────────────────────────────────────────────
echo "[column] cleanup 殘留 SITL / MAVProxy"
pkill -x arduplane 2>/dev/null || true
pkill -f sim_vehicle 2>/dev/null || true
[ "$BRIDGE" = "1" ] && { pkill -f mavproxy.py 2>/dev/null || true; }
sleep 2

# ── 逐台生成（單縱列）───────────────────────────────────────────────
# UAV1 = 長機(隊首) 生在基準點；UAV i 排在其正後方 i*SPACING 公尺處。
# 每台自己的 working dir + `-w` 清 eeprom → sitl_extra 等 defaults 才會真正吃進去
# （ArduPilot 只對「storage 內尚無」的參數載入 --defaults；共用 eeprom 會 race）。
echo "[column] 啟動 $N 台 arduplane（direct binary，單縱列）"
printf '  %-6s %-4s %-6s %-24s\n' "UAV" "-I" "TCP" "home(lat,lon,alt,hdg)"
for i in $(seq 0 $((N - 1))); do
  n=$((i + 1))
  tcp=$((5760 + i * 10))
  wd="$BASE/uav$n"; mkdir -p "$wd"; rm -f "$wd/eeprom.bin" "$wd/mav.parm"

  home=$(awk -v lat="$BLAT" -v lon="$BLON" -v alt="$BALT" -v hdg="$BHDG" \
             -v i="$i" -v s="$SPACING" -v cb="$COL_BEARING" \
    'BEGIN{
       pi=3.141592653589793;
       # 縱列方向：未指定 → 航向反向(排在長機後方)；指定 → 用固定方位角
       bearing = (cb=="") ? (hdg+180.0) : cb;
       br = bearing*pi/180.0;
       dist = i*s;
       dN = dist*cos(br); dE = dist*sin(br);
       dlat = dN/111320.0;
       dlon = dE/(111320.0*cos(lat*pi/180.0));
       printf "%.7f,%.7f,%s,%s", lat+dlat, lon+dlon, alt, hdg;
     }')

  printf '  UAV%-3d %-4d %-6d %s\n' "$n" "$i" "$tcp" "$home"
  ( cd "$wd" && exec "$AP" -w --model plane --home "$home" \
       -I"$i" --sysid "$n" ${DEF_LIST:+--defaults "$DEF_LIST"} ) \
       > "/tmp/sitl_uav$n.log" 2>&1 &
done

# ── 等 N 個 SITL TCP port 都 LISTEN 再宣告完成 ──────────────────────
echo "[column] 等待 $N 個 SITL TCP port 起來 (5760..$((5760 + (N - 1) * 10)))..."
up=0
for _t in $(seq 1 60); do
  up=0
  for i in $(seq 0 $((N - 1))); do
    ss -ltn 2>/dev/null | grep -q ":$((5760 + 10 * i)) " && up=$((up + 1))
  done
  [ "$up" -ge "$N" ] && break
  sleep 1
done
if [ "$up" -ge "$N" ]; then
  echo "[column] OK：$N 台全數上線（logs: /tmp/sitl_uav<N>.log）"
else
  echo "[column] WARN：只有 $up/$N 台 port 上線 —— 缺少者見 /tmp/sitl_uav<N>.log"
fi

WSL_IP="$(hostname -I | awk '{print $1}')"
echo "[column] WSL_IP=$WSL_IP"
echo "[column] AeroPlan 連線方式："
echo "         (a) 直接 TCP：SITL 分頁 → HIL/連線，連 tcp:127.0.0.1:5760（WSL2 localhost 轉發）"
echo "         (b) 想用 6760 fan-out：改用 BRIDGE=1 重跑，或另跑 bridge_mavproxy_swarm.sh $N"

# ── 選配：MAVProxy fan-out（AeroPlan 嵌入式蜂群 base_port 6760）──────
if [ "$BRIDGE" = "1" ]; then
  echo "[column] BRIDGE=1 → 啟動 MAVProxy fan-out（MAVROS 5860 / AeroPlan 6760 / node 6960）"
  bash "$BR/bridge_mavproxy_swarm.sh" "$N" > /tmp/bridge.log 2>&1 &
  sleep 3
fi

echo "[column] 全部啟動完成；wait 常駐（Ctrl+C 或 swarm_stop.sh 停止）"
wait
