#!/usr/bin/env bash
# stot_trial.sh — 跑「一趟」STOT 紅心命中,**假設 SITL 已經在跑**。
#
# 與 stot_kpi_loop.sh 的差別:那支每趟都重啟 SITL(95s)+ 完整逾時等待,適合無人
# 值守連跑;這支省掉重啟,適合「改一個參數 → 立刻看結果」的迭代。
# ⚠️ 一趟打完機體就消耗在地上了(expend_on_impact),要再跑一趟必須重啟 SITL。
#
# 用法: TRIAL=b1 PN=False bash stot_trial.sh
# 環境變數: PN / DIVE_PACE / GATE_R / N_LAT / N_VERT / PN_LEAD / DIVE_LAYER
#           TRACK_FRAC / ALIGN_TOL / DIVE_ANGLE / TRIAL(標籤) / CSV
# ⚠️ 不要 set -u:source ROS2 setup.bash 會踩到未設定的 AMENT_TRACE_SETUP_FILES。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
TRIAL="${TRIAL:-t1}"
CSV="${CSV:-/tmp/stot_kpi.csv}"
MISS_MAX="${MISS_MAX:-5}"
ALT_MAX="${ALT_MAX:-30}"
SPREAD_MAX="${SPREAD_MAX:-3.0}"
PN="${PN:-False}"
DIVE_PACE="${DIVE_PACE:-40.0}"
GATE_R="${GATE_R:-1200.0}"
N_LAT="${N_LAT:-4.0}"
N_VERT="${N_VERT:-3.0}"
PN_LEAD="${PN_LEAD:-1.0}"
DIVE_LAYER="${DIVE_LAYER:-12.0}"
TRACK_FRAC="${TRACK_FRAC:-1.0}"
DIVE_SPDW="${DIVE_SPDW:-1.0}"
ALIGN_TOL="${ALIGN_TOL:-40.0}"
DIVE_ANGLE="${DIVE_ANGLE:-10.0}"
LOG="${LOG:-/tmp/trial_${TRIAL}.log}"

[ -f "$CSV" ] || echo "trial,spread_s,max_miss_m,max_abs_alt_m,n_hit,pass,note" > "$CSV"

if [ "$(pgrep -x arduplane | wc -l)" -ne 6 ]; then
  echo "✗ SITL 不是 6 架(現在 $(pgrep -x arduplane | wc -l))—— 先跑 swarm_sitl_up.sh"
  exit 1
fi

echo "═══ TRIAL $TRIAL  PN=$PN pace=$DIVE_PACE layer=$DIVE_LAYER track=$TRACK_FRAC ═══"
# TOT_ONLY=1:蜂群已經在空中(上一趟沒打完就中止),只重跑 tot_node。
# 省掉重啟 SITL + 彈射 + 四散的 ~10 分鐘,迭代導引邏輯時用這個。
if [ "${TOT_ONLY:-0}" = "1" ]; then
  echo "[1/1] 蜂群已在空中,直接跑 tot_node"
else
echo "[1/3] 彈射起飛…"
( cd "$SR" && timeout 260 python3 jet_catapult_launch.py 5 90 ) > /tmp/trial_launch.log 2>&1
if ! grep -q "全員 GUIDED" /tmp/trial_launch.log; then
  echo "$TRIAL,,,,0,0,launch_failed" >> "$CSV"; echo "  ✗ 彈射失敗"; tail -5 /tmp/trial_launch.log; exit 1
fi
echo "[2/3] 四散進扇形…"
( cd "$SR" && timeout 420 python3 jet_rescatter.py 300 ) > /tmp/trial_scatter.log 2>&1
fi

echo "[3/3] STOT + 俯衝 → $LOG"
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash" 2>/dev/null
setsid nohup ros2 run sar_swarm tot_node --ros-args \
  -p tlat:=25.1406 -p tlon:=121.2325 -p talt:=400.0 -p approach_from:=180.0 \
  -p r_fan:=4000.0 -p half_angle:=25.0 -p vmin:=45.0 -p vmax:=100.0 \
  -p plan_frac:=0.9 -p margin:=40.0 -p perch_alt:=400.0 -p ground_alt:=-48.0 \
  -p dive_alt:=0.0 -p dive_layer:="$DIVE_LAYER" -p dive_angle:="$DIVE_ANGLE" \
  -p auto_ring:=True -p perch_alt_gate:=0.9 -p align_tol:="$ALIGN_TOL" \
  -p track_frac:="$TRACK_FRAC" -p dive_radius:=5.0 -p dive_pace:="$DIVE_PACE" \
  -p dive_spdweight:="$DIVE_SPDW" -p overshoot:=300.0 -p switch_r:=300.0 \
  -p dive_speed:=140.0 -p reach:=260.0 -p layer_m:=25.0 -p perch_reach:=260.0 \
  -p impact_reach:=25.0 -p pn_enable:="$PN" -p gate_r:="$GATE_R" \
  -p pn_n_lat:="$N_LAT" -p pn_n_vert:="$N_VERT" -p pn_lead:="$PN_LEAD" \
  -p csv:="/tmp/tot_${TRIAL}" \
  -p link_base:=6960 -p link_stride:=10 > "$LOG" 2>&1 < /dev/null &

ok=0
for w in $(seq 1 75); do          # 75 × 12s = 15 min 上限
  sleep 12
  if grep -q "俯衝命中紅心:全員命中" "$LOG"; then ok=1; break; fi
done
pkill -x tot_node 2>/dev/null
if [ "$ok" -ne 1 ]; then
  nh=$(grep -c "命中紅心 @" "$LOG")
  echo "$TRIAL,,,,$nh,0,timeout" >> "$CSV"
  echo "  ✗ 逾時(只命中 $nh/6);最後階段:"
  grep -E "★|就位對準環|交接終端" "$LOG" | tail -8
  exit 2
fi
MISS_MAX="$MISS_MAX" ALT_MAX="$ALT_MAX" SPREAD_MAX="$SPREAD_MAX" \
  TRIAL="$TRIAL" CSV="$CSV" TOT_LOG="$LOG" python3 "$SR/_kpi_parse.py"
echo "── 遙測基準 ──"; tail -1 "$CSV"
echo "── 真值基準 ──"; grep "真值基準" "$LOG" | tail -1
