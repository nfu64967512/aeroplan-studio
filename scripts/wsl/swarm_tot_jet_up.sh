#!/usr/bin/env bash
# swarm_tot_jet_up.sh — 蜂群終端俯衝齊射(TOT dive)【噴射尺度】。
# plane-jet 物理:巡航 ~40、俯衝可達 ~100 m/s;迴轉半徑大(~500m)→ 幾何全面放大。
# 前置:swarm_sitl_up.sh JET=1 已跑、6 機已起飛在空中(jet_prep.py)、機體端已放寬
#       AIRSPEED_MAX/TECS_SINK_MAX/PTCH_LIM_MIN(允許陡快俯衝)。
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash"
pkill -x tot_node 2>/dev/null || true
sleep 1
f() { case "$1" in *.*) echo "$1";; *) echo "$1.0";; esac; }

TLAT="${TLAT:-25.1406}"; TLON="${TLON:-121.2325}"; TALT="${TALT:-130}"
APPROACH="${APPROACH:-180}"; RFAN="${RFAN:-4000}"; HALF="${HALF:-25}"
VMIN="${VMIN:-45}"; VMAX="${VMAX:-100}"; PLAN="${PLAN:-0.9}"; MARGIN="${MARGIN:-45}"
# 俯衝(噴射尺度):高點 600m、pushover 1200m、俯衝指令 140(>頂速逼機頭下壓)
PERCH_ALT="${PERCH_ALT:-600}"; PUSHOVER_R="${PUSHOVER_R:-1200}"; DIVE_ALT="${DIVE_ALT:-15}"
DIVE_LAYER="${DIVE_LAYER:-12}"; OVERSHOOT="${OVERSHOOT:-2500}"; SWITCH_R="${SWITCH_R:-500}"
DIVE_SPEED="${DIVE_SPEED:-140}"; REACH="${REACH:-260}"; LAYER="${LAYER:-25}"
PERCH_REACH="${PERCH_REACH:-260}"; IMPACT_REACH="${IMPACT_REACH:-120}"

echo "[tot-jet] 目標=$TLAT,$TLON 扇形R=$RFAN ±$HALF° perch=${PERCH_ALT} pushover=${PUSHOVER_R} dive_speed=${DIVE_SPEED}"
exec ros2 run sar_swarm tot_node --ros-args \
  -p tlat:="$(f "$TLAT")" -p tlon:="$(f "$TLON")" -p talt:="$(f "$TALT")" \
  -p approach_from:="$(f "$APPROACH")" -p r_fan:="$(f "$RFAN")" -p half_angle:="$(f "$HALF")" \
  -p vmin:="$(f "$VMIN")" -p vmax:="$(f "$VMAX")" -p plan_frac:="$(f "$PLAN")" -p margin:="$(f "$MARGIN")" \
  -p perch_alt:="$(f "$PERCH_ALT")" -p pushover_r:="$(f "$PUSHOVER_R")" -p dive_alt:="$(f "$DIVE_ALT")" \
  -p dive_layer:="$(f "$DIVE_LAYER")" -p overshoot:="$(f "$OVERSHOOT")" -p switch_r:="$(f "$SWITCH_R")" \
  -p dive_speed:="$(f "$DIVE_SPEED")" -p reach:="$(f "$REACH")" -p layer_m:="$(f "$LAYER")" \
  -p perch_reach:="$(f "$PERCH_REACH")" -p impact_reach:="$(f "$IMPACT_REACH")"
