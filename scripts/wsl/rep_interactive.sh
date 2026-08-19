#!/usr/bin/env bash
# rep_interactive.sh — 互動路徑(3100 m 噴射剖面)的**同組態重複航次**。
#
# ══ 為什麼需要這支 ══════════════════════════════════════════════════════
# AP5 單趟拿到命中時間散布 0.95 s(門檻 3 s),但**單趟沒有診斷力**:
# stot-refuted-changes 記載 B0 同組態三趟是 17.50 / 30.72 / 10.90 s
# ⇒ 散布 = 19.7 ± 10.1 s,**任何差距小於 ~20 s 的單次比較都不算數**。
# 要判定 2026-08-10 這一輪(3100 m 剖面 + 軟交接 + 噴射組態)真的有效,
# 必須同組態跑 ≥3 趟。這支就是那個工具。
#
# ⚠️⚠️ 這支**不碰 AeroPlan**。它直接發 /swarm/tot_target ——
#    那是 AeroPlan 右鍵送出的**同一則訊息**(見 integrations/rclick_strike.py
#    的 build_target_payload),互動路徑本身已在 AP3/AP5 端到端驗證過。
#    這裡要量的是**組態**,不是 GUI;拿掉 GUI 才能讓每趟的 payload 逐位元相同。
#
# ⚠️ 每趟都**完整重建 SITL + ROS2**:tot_node 的 expend_on_impact=True,
#    命中的機體會被消耗掉,不重建就沒有飛機可以飛第二趟。
#
# ⚠️ 失敗的趟次**照樣記錄**,不可以讓它消失 —— 只有失敗會消失的話,
#    成功率會被灌高,方向永遠樂觀(stot_rescore_all.py 的同一條教訓)。
#
# 用法:
#   bash rep_interactive.sh                 # 3 趟,預設目標
#   TRIALS=5 bash rep_interactive.sh
#   PREFIX=REPB bash rep_interactive.sh     # 換一組輸出目錄名
set +u          # ⚠️ set -u 會被 ROS2 的 setup.bash 打爆(未綁定的 AMENT_TRACE_SETUP_FILES)
set -o pipefail

SR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HOME/fixwing_ws/runs"
ART="${ART:-/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts}"

TRIALS="${TRIALS:-3}"
PREFIX="${PREFIX:-REP}"
N="${N:-2}"

# ── 目標:與 AP5 **逐位元相同**(含 approach_from) ────────────────────
# ⚠️ approach_from 一定要明示。不給的話 tot_node 走 auto_approach(由蜂群質心
#    反推),而質心每趟都不一樣 ⇒ 扇形方位每趟不同 ⇒ 這就不是單變數對照了。
TGT_LAT="${TGT_LAT:-25.1261649}"
TGT_LON="${TGT_LON:-120.6156480}"
TGT_APPROACH="${TGT_APPROACH:-94.8}"
TGT_AMSL="${TGT_AMSL:-0.0}"          # 海面。⚠️ 不可省 —— SRTM 在水域沒資料
PAYLOAD="{\"lat\": $TGT_LAT, \"lon\": $TGT_LON, \"approach_from\": $TGT_APPROACH, \"amsl\": $TGT_AMSL}"

# ⚠️ 機體改裝參數要明講。swarm_sitl_up.sh **從不自己設 JET=1**,
#    裸跑會拿到原廠泡棉機而毫無錯誤訊息(swarm_ros2_up.sh 的守門會擋,但
#    我們不該讓它擋 —— 這裡直接給對)。
export JET=1
export JET_CDP="${JET_CDP:-0.01642}"
export LOC="${LOC:-TAOYUAN}"

# ⚠️⚠️⚠️ **JET_THRMAX 刻意不設。**
#
# `swarm_sitl_up.sh:77-78` 原文:「推重比 → THR_MAX。calculate_forces() 算的是
# servo 輸出 × thrust_scale,而 servo 上限就是 THR_MAX ⇒ THR_MAX=19 等於
# 可用推力只剩 19%。」
#
# 25 是**巡航設計值**。批次路徑可以用它,是因為它有 `jet_catapult_launch.py`
# 在爬升段動態把 THR_MAX 拉到 35(run_west.sh 的 CLIMB_THR_MAX)——
# **互動路徑沒有那一段**。照抄 25 等於要飛機用巡航油門上限去起飛爬升。
#
# 實測(RPE1/RPE2,2026-08-10):兩機起飛爬到 53/54 m 之後,**40 秒內沉回地面**
#   RPE2 t=43 s:UAV1 地速 11.5、UAV2 地速 0.1 —— 都在地上
# 而 AP3/AP5 飛得起來(13 分鐘、2/2 命中),是因為那次的臨時 bring-up
# **漏了 JET_THRMAX** ⇒ THR_MAX 停在 plane-jet 原廠的 100。
# ⇒ 也就是說 **AP5 是在 THR_MAX=100 之下飛出來的**,不是完整的噴射組態。
#
# 不設 ⇒ swarm_sitl_up.sh:208 的 `if [ -n "${JET_THRMAX:-}" ]` 為假 ⇒ 整段跳過
#      ⇒ THR_MAX 維持 100,與 AP3/AP5 逐位元相同。
# ⚠️ 要試 25 的話,必須**同時**補上爬升段的推力提升,否則飛機起不來。
unset JET_THRMAX

# 每趟的硬時限(從發目標算起)。AP5 實測 772 s;留 2 倍餘裕。
STRIKE_TIMEOUT="${STRIKE_TIMEOUT:-1500}"

# ══ 發目標的時機:**條件式閘門,不是固定延遲** ══════════════════════════
# ⚠️⚠️ REP1 用固定 25 s 就發,結果兩機在 6 秒內全被觸水終結,而當下地速 100 m/s:
#         t=0.0 uav=1 alt=-30.252 gs=100.4 agl= 1.8
#         t=0.0 uav=2 alt=-36.089 gs=105.8 agl=-4.0
#    真因是**飛控的高度估計還沒收斂** —— 剛 arm/起飛那幾十秒 EKF 原點還在沉澱,
#    relative_alt 回報 −30 m。tot_node 端已經補了「解除保險」閂鎖擋這一類,
#    但航次本身也不該在估計還沒穩的時候開打。
# ⚠️ 另一頭也不能久等:AP4 的教訓 —— 編隊層把飛機壓在 patrol_alt 80 m
#    (地形 32 m),久滯會掉下去。tot_node 接手後才會爬到 perch。
# ⇒ 等「兩機都 armed 且高度估計已穩定」才發,上限 READY_MAX_S。
#
# ⚠️⚠️⚠️ **不要用「飛多高」當門檻。** RPB1 用 120 m、RPC1 用 60 m,兩次都逾時。
#    (RPB1 的 120 m 還多錯一層:編隊層 `patrol_alt` 只有 80 m ——
#     `master_node.py:43`、`swarm_ros2_up.sh` 的 `patrol_alt:=${PATROL_ALT:-…80.0}`。
#     這是本專案反覆踩的同一類錯:把門檻設在物理上到不了的地方,
#     如 reach=260 對 35 km 扇形、perch_alt_gate 要 360 m 而飛機卡在 30–80 m。)
#    實測(measure_climb,起飛後逐 5 s 量 180 s):
#
#        t(s)   UAV1_rel   UAV2_rel   UAV1_amsl   UAV2_amsl
#           0     65.787     -2.003      105.27       47.52
#          15    -24.891    -19.501       23.90       19.69
#          60    -17.658    -22.113       32.83       29.46
#         180     -8.265    -14.664       42.22       36.33
#
#    桃園跑道 +32 m AMSL ⇒ **三分鐘下來兩機一直在跑道高度上下浮沉,根本沒爬升**。
#    真因(兩個疊在一起,都在編隊層):
#      (a) 編隊層下 NOMINAL 14 / WAIT 10 / CATCH 20 m/s,而噴射機的
#          AIRSPEED_MIN 是 **25** ⇒ 每一發 DO_CHANGE_SPEED 都被 MAV_RESULT=4
#          整發拒收(catch-speed-rejected-by-fc 的另一端:那次太快,這次太慢)
#      (b) backup/wingman 以 1 Hz 重發 DO_REPOSITION,而每一發都會
#          `set_target_altitude_current()` 把高度目標重錨在當下高度
#          ⇒ 高度需求永遠累積不起來(tot_node 的 `_goto` 用航點鎖存修過同一個病,
#            編隊層沒有)
#    ⇒ 「等它爬到 X 公尺」這件事,編隊層**做不到**。門檻設多低都一樣。
#
# ⇒ 改成量「**高度估計可不可信**」——那才是 REP1 真正缺的東西:
#    REP1 的壞輸入是 EKF 未收斂(rel_alt −36 m 而地速 106 m/s),
#    特徵是**劇烈跳動**,不是「低」。所以等它**穩定**下來就好。
#    條件:兩機都 armed,且 AMSL 連續 READY_STABLE_N 次取樣的極差 < READY_BAND m。
READY_BAND="${READY_BAND:-15}"     # 連續取樣的 AMSL 極差門檻(m)
READY_STABLE_N="${READY_STABLE_N:-4}"
# 180 → 300:門檻從「AMSL 穩定」(地面上 4 s 就成立)改成「爬到 250 m」,
# 需要的時間從秒級變成分鐘級。逾時等於整趟報廢,寧可給足餘裕。
READY_MAX_S="${READY_MAX_S:-300}"

# ⚠️⚠️ **只等「高度估計穩定」是不夠的 —— 飛機停在跑道上時它永遠成立。**
#    RPG/RPH 兩批(2026-08-10)在這裡整批報廢,指紋一模一樣:
#
#        UAV1  gs_max=0.0  alt_max=0.1  樣本=6290   ← 樣本數正常,遙測沒斷
#        UAV2  gs_max=197.7 alt_max=202.7 樣本=6290
#
#    UAV1 整趟停在跑道上一步沒動。閘門在 **4 秒**就通過(地面上 AMSL 當然穩定),
#    目標一發出去,tot_node 立刻用 DO_REPOSITION 把飛機從 TAKEOFF 拉進 GUIDED
#    —— 而 **ArduPlane 在地面上的 GUIDED 不會執行起飛**,起飛就此被取消。
#
# ⚠️ 這個坑是 CHANGE_MODE 修好之後才**露出來**的,不是它造成的:
#    在那之前 TAKEOFF 模式會拒收位置指令,反而意外保護了起飛
#    (RPF 的 UAV2 因此還飛得起來,只是飛去繞圈)。
#    ⇒ 兩件事都得修:指令要能改模式,而**指令不可以在飛機還沒離地時發出去**。
#
# ⚠️ 為什麼「等爬到 X 公尺」這次做得到,而註解上方說編隊層做不到:
#    爬升是**飛控自己的 TAKEOFF 模式**給的(swarm_takeoff.sh:5「自動滾行爬升到
#    TKOFF_ALT;爬過 guided_min_alt(15m) 後節點接手」),不是編隊層給的。
#    佐證:`jet_catapult_launch.py:238`「DO_REPOSITION 在 TAKEOFF 模式下不起作用:
#    那個模式只爬到 TKOFF_ALT 就平飛」—— 批次路徑早就知道,互動路徑沒抄到。
#
# ⚠️⚠️ **門檻只能取 30,不能抄批次路徑的 250。** 我一度設 250(理由是
#    `jet_verify.py:220`「TKOFF_ALT 設 300,這裡等到 250」),那是**錯的**:
#    批次路徑沒有編隊層,飛機可以一路在 TAKEOFF 模式爬到 300;
#    互動路徑的編隊層會在 **guided_min_alt=15 m** 就用 `set_mode` 接手 GUIDED,
#    然後把飛機帶去 `patrol_alt`(= `${PATROL_ALT:-${FORM_ALT:-80}}`,實際 **50 m**)。
#    ⇒ 250 m **永遠到不了**,閘門必定逾時 —— 那正是 RPB1/RPC1 犯過的
#      「不可達門檻」,差點犯第四次。
#    30 m 的位置:遠高於 15 m 的接手點(確定離地),又遠低於 50 m 的巡邏高度
#    (確定到得了)。
READY_AGL="${READY_AGL:-30}"       # 離地高(m);低於此不算起飛完成

_wait_stable() {                     # 回傳 0=高度估計已穩定**且全員已離地**
  # ⚠️⚠️ **速度就是正確性的一部分。** 第一版每輪對每機各發 2 次
  #    `ros2 topic echo --once`(每次 ~2 s 的 python 啟動)⇒ 4 輪要 ~40 s。
  #    而編隊層在起飛後 ~40 s 就把飛機沉回地面(見上面的 JET_THRMAX 說明)
  #    ⇒ **閘門本身的延遲把窗口吃掉了**。
  # ⇒ 改成每機**只開一個**常駐 echo 把樣本寫進檔案,主迴圈只讀檔。
  #    一趟總共 N 次 python 啟動,不是 N × 2 × 輪數。
  local t i pids=""
  for i in $(seq 1 "$N"); do
    : > "/tmp/rep_amsl_$i.txt"
    ( timeout "$READY_MAX_S" ros2 topic echo \
        "/uav$i/mavros/global_position/global" 2>/dev/null \
      | grep --line-buffered '^altitude:' | awk '{print $2; fflush()}' \
      >> "/tmp/rep_amsl_$i.txt" ) &
    pids="$pids $!"
    # 離地高走 rel_alt(相對 HOME),不自己拿 AMSL 減地形 ——
    # agl-datum-false-kill 的教訓:兩個基準不同步會平移幾十 m。
    : > "/tmp/rep_agl_$i.txt"
    ( timeout "$READY_MAX_S" ros2 topic echo \
        "/uav$i/mavros/global_position/rel_alt" 2>/dev/null \
      | grep --line-buffered '^data:' | awk '{print $2; fflush()}' \
      >> "/tmp/rep_agl_$i.txt" ) &
    pids="$pids $!"
  done
  local rc=1
  for t in $(seq 1 "$READY_MAX_S"); do
    local stable=1 airborne=1
    for i in $(seq 1 "$N"); do
      tail -n "$READY_STABLE_N" "/tmp/rep_amsl_$i.txt" 2>/dev/null \
        | awk -v b="$READY_BAND" -v n="$READY_STABLE_N" '
            NR==1{lo=$1;hi=$1} {if($1<lo)lo=$1; if($1>hi)hi=$1}
            END{exit !(NR>=n && hi-lo < b+0)}' || stable=0
      # ⚠️ 要**連續 READY_STABLE_N 筆都**過門檻,不是看最後一筆。
      #    舊測試的反對理由是「不能拿一個壞量去判斷它自己好了沒」——
      #    這一條正面回答它:單一尖峰不夠,而且 AMSL 穩定度仍然是**必要**條件,
      #    兩個要同時成立。壞的 rel_alt(實測 −36 m)只會讓閘門**不放行**,
      #    不會讓它誤放行。
      tail -n "$READY_STABLE_N" "/tmp/rep_agl_$i.txt" 2>/dev/null \
        | awk -v a="$READY_AGL" -v n="$READY_STABLE_N" '
            NR==1{lo=$1} {if($1+0<lo+0)lo=$1}
            END{exit !(NR>=n && lo+0 >= a+0)}' || airborne=0
    done
    if [ "$stable" = "1" ] && [ "$airborne" = "1" ]; then
      echo "[$TAG]   已就緒(AMSL 連續 ${READY_STABLE_N} 筆極差 < ${READY_BAND} m;"\
"全員離地 ≥ ${READY_AGL} m,等了 ${t} s)"
      for i in $(seq 1 "$N"); do
        printf '[%s]     UAV%s agl=%s m\n' "$TAG" "$i" \
          "$(tail -n 1 "/tmp/rep_agl_$i.txt" 2>/dev/null)"
      done
      rc=0
      break
    fi
    sleep 1
  done
  if [ "$rc" != "0" ]; then
    echo "[$TAG]   ✗ 逾時前狀態:stable=$stable airborne=$airborne" >&2
    for i in $(seq 1 "$N"); do
      printf '[%s]     UAV%s agl=%s m\n' "$TAG" "$i" \
        "$(tail -n 1 "/tmp/rep_agl_$i.txt" 2>/dev/null)" >&2
    done
  fi
  for p in $pids; do kill "$p" 2>/dev/null; done
  return "$rc"
}

source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$HOME/fixwing_ws/install/setup.bash" >/dev/null 2>&1

for _v in READY_BAND READY_STABLE_N READY_MAX_S READY_AGL STRIKE_TIMEOUT TRIALS PREFIX N; do
  if [ -z "$(eval "printf '%s' \"\${$_v}\"")" ]; then
    echo "[rep] ✗ 內部錯誤:變數 $_v 是空的(刪掉舊變數後忘了改引用?)" >&2
    exit 1
  fi
done

_down() {
  bash "$SR/swarm_ros2_down.sh" >/dev/null 2>&1
  for p in $(pgrep -f '[t]ot_node --ros-args'); do kill -TERM "$p" 2>/dev/null; done
  for p in $(pgrep -f '[m]avros_node'); do kill -9 "$p" 2>/dev/null; done
  sleep 2
  for p in $(pgrep -f '[t]ot_node --ros-args'); do kill -9 "$p" 2>/dev/null; done
  bash "$SR/swarm_sitl_down.sh" >/dev/null 2>&1
  sleep 3
}

_wait_ports() {                      # 四路分流都要 LISTEN
  local t p ok
  for t in $(seq 1 90); do
    ok=1
    for p in 5860 6760 6860 6960; do
      ss -ltn 2>/dev/null | grep -q ":$p " || ok=0
    done
    [ "$ok" = "1" ] && return 0
    sleep 2
  done
  return 1
}

# ══ 自我檢查:`bash -n` 抓不到的兩類 ═══════════════════════════════════
# ⚠️⚠️ RPD 實測:我把函式從 `_wait_airborne` 改名成 `_wait_stable`,
#    **呼叫點忘了改**,而且訊息還引用已刪除的 `$READY_ALT`。
#    症狀是 `等兩機離地(rel_alt ≥ ⟨空⟩ m)` 然後每趟直接記失敗 ——
#    `bash -n` 只驗語法,**未定義的函式與空變數它都看不到**。
#    ⇒ 開跑前自己驗一次:要用到的函式都存在、要印的變數都非空。
for _fn in _down _wait_ports _wait_stable; do
  if ! declare -F "$_fn" >/dev/null 2>&1; then
    echo "[rep] ✗ 內部錯誤:函式 $_fn 不存在(改名後忘了改呼叫點?)" >&2
    exit 1
  fi
done

echo "═══════════════════════════════════════════════════════════════"
echo "[rep] 同組態重複航次:$TRIALS 趟 / N=$N / 前綴 $PREFIX"
echo "[rep] 目標 payload(每趟逐位元相同):"
echo "[rep]   $PAYLOAD"
echo "[rep] ⚠️ 單趟散布沒有診斷力(B0 同組態三趟 17.5/30.7/10.9,±10 s)"
echo "═══════════════════════════════════════════════════════════════"

# ⚠️⚠️ **每一批開跑前必須部署。** 本腳本先前完全沒有這一步,
#    ⇒ 每趟跑的都是 WSL 裡上一次留下的舊副本,repo 改了也不會生效,
#      而且**沒有任何訊息**(colcon 判定「不需重建」時同樣靜默)。
#    實例:RPF 那一批(2026-08-10)在 repo 端修好 DO_REPOSITION 的
#    CHANGE_MODE 旗標後,若沒有這一步,下一批會原封不動再失敗一次。
#    deploy_build.sh 自己會逐檔比 md5、乾淨重建、再比 build/ 一次,
#    對不上就非零退出 —— 所以這裡只要不吞它的 rc。
echo "[rep] 部署 repo → WSL 並重建(md5 驗證)…"
if ! bash "$SR/deploy_build.sh" > "$RUNS/${PREFIX}_deploy.log" 2>&1; then
  echo "[rep] ✗ 部署失敗 —— 整批中止(跑舊碼比不跑更糟)" >&2
  tail -20 "$RUNS/${PREFIX}_deploy.log" >&2
  exit 1
fi
grep -E "部署一致|✅|✗" "$RUNS/${PREFIX}_deploy.log" | tail -3

SUMMARY="$RUNS/${PREFIX}_summary.txt"
: > "$SUMMARY"

for k in $(seq 1 "$TRIALS"); do
  TAG="${PREFIX}${k}"
  OUT="$ART/$TAG"
  mkdir -p "$OUT"
  echo
  echo "───────────────────────────── [$TAG] $(date '+%H:%M:%S')"

  _down
  echo "[$TAG] 起 SITL…"
  setsid env N="$N" SWARM_COUNT="$N" bash "$SR/swarm_sitl_up.sh" \
      > "$RUNS/${TAG}_sitl.log" 2>&1 &
  if ! _wait_ports; then
    echo "[$TAG] ✗ SITL 分流埠沒起完 —— 記為失敗" | tee -a "$SUMMARY"
    continue
  fi

  echo "[$TAG] 起 ROS2…"
  setsid env N="$N" SWARM_COUNT="$N" MANUAL=1 bash "$SR/swarm_ros2_up.sh" \
      > "$RUNS/${TAG}_ros2.log" 2>&1 &
  _rdy=0
  for t in $(seq 1 120); do
    grep -q "ready_for_target=true" "$RUNS/${TAG}_ros2.log" 2>/dev/null && { _rdy=1; break; }
    grep -qE "✗ 機體身分|ERROR" "$RUNS/${TAG}_ros2.log" 2>/dev/null && break
    sleep 2
  done
  if [ "$_rdy" != "1" ]; then
    echo "[$TAG] ✗ tot_node 沒有進入 ready_for_target —— 記為失敗" | tee -a "$SUMMARY"
    tail -6 "$RUNS/${TAG}_ros2.log" 2>/dev/null
    continue
  fi
  grep -E "機體身分" "$RUNS/${TAG}_ros2.log" | tail -1

  # ══ 生效值契約(起飛前的最後一道)═══════════════════════════════════
  # ⚠️⚠️ 使用者 2026-08-11 定調:「靜默失敗與吃到舊參數不可再發生」。
  #    這一關把**節點層**生效值(vmin/vmax/dive_speed/dive_pace/gate_r/
  #    formation_alt/patrol_alt)與飛控 PARM 一起讀回來,對照契約表的
  #    區間與**泡棉禁區**。
  # ⚠️ 與 preflight_check.py 是互補**不是二選一**:後者走 pymavlink,
  #    構不到 ROS2 節點參數 —— 上面那 8 項先前完全沒有任何檢查覆蓋,
  #    而 tot_node 的 declare_parameter 預設值全部是泡棉時代的,
  #    任何一條餵值路徑斷掉都會靜靜退回去跑原廠常數。
  # ⚠️ 快照與 CSV/log/geom 同級收進產物 —— 事後才回答得了
  #    「那一趟到底跑的是什麼」,而不是靠記憶或猜。
  # ⚠️⚠️ `if ! cmd | tee …` 取到的是 **tee** 的 rc 而不是 cmd 的,
  #    那會讓這道閘門看起來永遠通過。所以用 PIPESTATUS 明確取第一段。
  echo "[$TAG] 生效值契約…"
  COUNT="$N" SWARM_COUNT="$N" READY_AGL="$READY_AGL" \
    python3 "$SR/contract_verify.py" --snapshot "$OUT/contract_$TAG.json" \
    2>&1 | tee "$RUNS/${TAG}_contract.log"
  _crc="${PIPESTATUS[0]}"
  if [ "$_crc" != "0" ]; then
    # ⚠️ 整批中止而不是跳過這一趟:組態是**逐趟相同**的,第一趟不合契約
    #    就代表每一趟都不合。逐趟跳過只會燒掉 TRIALS 次一模一樣的失敗,
    #    還把「組態錯了」偽裝成「航次失敗」。同 deploy 那一關的處置。
    echo "[$TAG] ✗ 生效值契約未通過 —— **整批中止**" | tee -a "$SUMMARY"
    echo "[rep]   跑一趟不符契約的航次,產出的數字不能用來下結論。" >&2
    echo "[rep]   逐項反證與可貼的重跑指令見上方輸出;" >&2
    echo "[rep]   快照留在 $OUT/contract_$TAG.json。" >&2
    exit 1
  fi

  echo "[$TAG] 起飛…"
  bash "$SR/swarm_takeoff.sh" $(seq 1 "$N") > "$RUNS/${TAG}_takeoff.log" 2>&1
  grep -c "TAKEOFF + ARMED ✓" "$RUNS/${TAG}_takeoff.log" | xargs -I{} echo "[$TAG]   armed {}/$N"
  if [ "$(grep -c 'TAKEOFF + ARMED ✓' "$RUNS/${TAG}_takeoff.log")" -lt "$N" ]; then
    echo "[$TAG]   有機體沒起飛,重試一次…"
    bash "$SR/swarm_takeoff.sh" $(seq 1 "$N") >> "$RUNS/${TAG}_takeoff.log" 2>&1
  fi

  echo "[$TAG] 等高度估計穩定(AMSL 連續 ${READY_STABLE_N} 筆極差 < ${READY_BAND} m,上限 ${READY_MAX_S}s)…"
  if ! _wait_stable; then
    echo "[$TAG] ✗ ${READY_MAX_S}s 內高度估計沒有穩定下來 —— 記為失敗" | tee -a "$SUMMARY"
    continue
  fi
  echo "[$TAG] 發目標…"
  timeout 40 ros2 topic pub --once -w 1 /swarm/tot_target std_msgs/String \
      "{data: '$PAYLOAD'}" > "$RUNS/${TAG}_pub.log" 2>&1
  _prc=$?
  if [ "$_prc" != "0" ]; then
    echo "[$TAG] ✗ 目標沒送出去(rc=$_prc,124=沒有訂閱者)—— 記為失敗" | tee -a "$SUMMARY"
    continue
  fi

  echo "[$TAG] 等結案(上限 ${STRIKE_TIMEOUT}s)…"
  _done=0
  for t in $(seq 1 "$((STRIKE_TIMEOUT / 5))"); do
    grep -q "俯衝結案" "$RUNS/tot_standby.log" 2>/dev/null && { _done=1; break; }
    sleep 5
  done

  cp "$RUNS/tot_standby.log" "$OUT/tot_standby.log" 2>/dev/null
  cp "$RUNS/tot_standby.csv" "$OUT/tot_${TAG}.csv" 2>/dev/null
  # ⚠️⚠️ **geom 一定要一起收。** 它是 `stot_score.py` / `impact_feasibility.py` /
  #    `spread_growth.py` / `_tot_plot.py` / `_dive_plot.py` 五支工具的必要輸入
  #    (格式 `kind,lat,lon`,取 `kind==target`),由 tot_node 的
  #    `_write_geom_csv()` 在目標被接受時寫出。
  # ⚠️ 漏收它的後果不是「少一個檔」,而是**評分工具整支跑不起來**
  #    ⇒ 只好去讀結案訊息的鎖存值,而那個值是**真最近點的上界**
  #      (誤差 ≤ 一個取樣步;末端 120 m/s ⇒ 十幾公尺)。
  #    RPJ 那一批就是這樣被讀成「誤差 26–32 m」,實際內插量只有 0.03–0.86 m。
  cp "$RUNS/tot_standby_geom.csv" "$OUT/tot_${TAG}_geom.csv" 2>/dev/null

  if [ "$_done" != "1" ]; then
    echo "[$TAG] ✗ 逾時,沒有結案 —— 記為失敗" | tee -a "$SUMMARY"
    continue
  fi
  {
    echo "── $TAG"
    grep -E "全員抵扇形" "$RUNS/tot_standby.log" | tail -1
    grep -E "全員對準內切" "$RUNS/tot_standby.log" | tail -1
    grep -E "俯衝結案" "$RUNS/tot_standby.log" | tail -1
    grep -E "真值基準" "$RUNS/tot_standby.log" | tail -1
  } | tee -a "$SUMMARY"

  # ── 正確量尺:當場評分,不要留給事後 ────────────────────────────
  # 結案訊息的「最近水平距」是鎖存凍結值(上界);`stot_score.py` 會在取樣點
  # 之間 `seg_closest` 內插,才是真最近點。把它寫進摘要,免得又去讀錯的數字。
  if [ -f "$OUT/tot_${TAG}_geom.csv" ]; then
    {
      echo "   ── $TAG 內插評分(stot_score.py:d1 才是真誤差)"
      python3 "$SR/stot_score.py" "$OUT/tot_${TAG}.csv" \
              --log "$OUT/tot_standby.log" 2>&1 \
        | grep -E "首航次:|觸地基準|① |② |③ |判定:" | sed 's/^/   /'
    } | tee -a "$SUMMARY"
  else
    echo "   ⚠️ $TAG 沒有 geom.csv —— 無法內插評分(只剩鎖存上界可讀)" \
      | tee -a "$SUMMARY"
  fi
done

echo
echo "═══════════════════════════════════════════════════════════════"
echo "[rep] 全部完成。逐趟摘要:$SUMMARY"
echo "[rep] ⚠️ 判讀時記得:單趟散布雜訊 ±10 s,要看三趟的分布不是單一值。"
echo "═══════════════════════════════════════════════════════════════"
cat "$SUMMARY"
