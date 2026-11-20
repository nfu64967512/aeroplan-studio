#!/usr/bin/env bash
# stot_kpi_loop.sh — STOT 六機命中 KPI 的無人值守重複試驗。
#
# 每一趟(trial)都是完整的:重啟 SITL → 彈射起飛 → 四散 → tot_node 俯衝 → 判定。
# 為什麼每趟都要重啟:命中即消耗(expend_on_impact),機體會留在地面拉不起來,
# 不重啟的話第二趟就湊不齊六機(實測 5/6 躺在地上 → ALIGN 永遠等不到)。
#
# KPI(三項全過才算成功):
#   1) 六機水平命中誤差都 <= MISS_MAX (預設 30 m)
#   2) 六機命中高度都 <= ALT_MAX      (預設 30 m,離紅心地表)
#   3) 六機命中時間散布 <= SPREAD_MAX (預設 3.0 s)
#
# 結果逐趟寫入 /tmp/stot_kpi.csv,並印出累計成功率。
# 用法: bash stot_kpi_loop.sh [趟數]
# ⚠️ 不可加 -u:本腳本要 source ROS2 的 setup.bash,它會讀未設定的
#    AMENT_TRACE_SETUP_FILES,-u 之下整個腳本會在第一趟就中止(實測踩到)。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
TRIALS="${1:-50}"
MISS_MAX="${MISS_MAX:-5}"
ALT_MAX="${ALT_MAX:-30}"
SPREAD_MAX="${SPREAD_MAX:-3.0}"
CSV="${CSV:-/tmp/stot_kpi.csv}"
DIVE_PACE="${DIVE_PACE:-40.0}"
PN="${PN:-True}"
GATE_R="${GATE_R:-1200.0}"
N_LAT="${N_LAT:-4.0}"
N_VERT="${N_VERT:-3.0}"
PN_LEAD="${PN_LEAD:-1.0}"

[ -f "$CSV" ] || echo "trial,spread_s,max_miss_m,max_abs_alt_m,n_hit,pass,note" > "$CSV"

for n in $(seq 1 "$TRIALS"); do
  echo "════════ TRIAL $n ════════"
  pkill -x tot_node 2>/dev/null
  bash "$SR/swarm_sitl_down.sh" >/dev/null 2>&1
  pkill -x mavproxy.py 2>/dev/null
  sleep 5
  # SITL 起來(背景常駐),等 bridge 做完 param-FTP 才會接受 client
  ( cd "$SR" && JET=1 LAUNCH=catapult AIRSPEED_MAX_CAP=150 LOC=TAOYUAN \
      bash "$SR/swarm_sitl_up.sh" >/tmp/kpi_sitl.log 2>&1 ) &
  sleep 95
  if [ "$(pgrep -x arduplane | wc -l)" -ne 6 ]; then
    echo "$n,,,,0,0,sitl_up_failed" >> "$CSV"; echo "  ✗ SITL 起不來"; continue
  fi
  ( cd "$SR" && timeout 260 python3 jet_catapult_launch.py 5 90 ) >/tmp/kpi_launch.log 2>&1
  if ! grep -q "全員 GUIDED" /tmp/kpi_launch.log; then
    echo "$n,,,,0,0,launch_failed" >> "$CSV"; echo "  ✗ 彈射失敗"; continue
  fi
  ( cd "$SR" && timeout 420 python3 jet_rescatter.py 300 ) >/dev/null 2>&1

  source /opt/ros/humble/setup.bash
  source "$HOME/fixwing_ws/install/setup.bash" 2>/dev/null
  setsid nohup ros2 run sar_swarm tot_node --ros-args \
    -p tlat:=25.1406 -p tlon:=121.2325 -p talt:=400.0 -p approach_from:=180.0 \
    -p r_fan:=4000.0 -p half_angle:=25.0 -p vmin:=45.0 -p vmax:=100.0 \
    -p plan_frac:=0.9 -p margin:=40.0 -p perch_alt:=400.0 -p ground_alt:=-48.0 \
    -p dive_alt:=0.0 -p dive_layer:=12.0 -p dive_angle:=10.0 -p auto_ring:=True \
    -p perch_alt_gate:=0.9 -p align_tol:=40.0 -p track_frac:=0.50 \
    -p dive_radius:=5.0 -p dive_pace:="$DIVE_PACE" -p dive_spdweight:=0.0 \
    -p overshoot:=300.0 -p switch_r:=300.0 -p dive_speed:=140.0 -p reach:=260.0 \
    -p layer_m:=25.0 -p perch_reach:=260.0 -p impact_reach:=25.0 \
    -p pn_enable:="$PN" -p gate_r:="$GATE_R" -p pn_n_lat:="$N_LAT" \
    -p pn_n_vert:="$N_VERT" -p pn_lead:="$PN_LEAD" \
    -p link_base:=6960 -p link_stride:=10 > /tmp/kpi_tot.log 2>&1 < /dev/null &

  ok=0
  for w in $(seq 1 55); do
    sleep 12
    # 固定哨兵「★★ 俯衝結案:」= 「跑完了」(成功與否交給 _kpi_parse.py)。
    # 四種結案形狀都有它;第二段只為了讀舊 log。見 stot_trial.sh 與
    # sar_swarm_patches/tot_node.py 的 CLOSEOUT_SENTINEL。
    if grep -qE "★★ 俯衝結案:|俯衝命中紅心:全員命中" /tmp/kpi_tot.log; then ok=1; break; fi
  done
  pkill -x tot_node 2>/dev/null
  if [ "$ok" -ne 1 ]; then
    nh=$(grep -c "命中紅心 @" /tmp/kpi_tot.log)
    echo "$n,,,,$nh,0,timeout" >> "$CSV"; echo "  ✗ 逾時(只命中 $nh/6)"; continue
  fi
  MISS_MAX="$MISS_MAX" ALT_MAX="$ALT_MAX" SPREAD_MAX="$SPREAD_MAX" \
    TRIAL="$n" CSV="$CSV" python3 "$SR/_kpi_parse.py"
  tail -1 "$CSV"
  python3 - "$CSV" <<'PY'
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1]))]
if rows:
    p = sum(1 for r in rows if r["pass"] == "1")
    print("  ▶ 累計成功率 %d/%d = %.1f%%" % (p, len(rows), 100.0 * p / len(rows)))
PY
done
