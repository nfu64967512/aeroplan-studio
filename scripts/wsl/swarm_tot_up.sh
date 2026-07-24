#!/usr/bin/env bash
# swarm_tot_up.sh — 蜂群同時抵達(TOT)協同層【ROS2 管道 ②】。
#
# 與編隊跟隨(swarm_ros2_up.sh)互斥:TOT 是不同任務模式,由 tot_node 獨佔導引。
# 前置:swarm_sitl_up.sh 已在跑(分流 5860/6760 已 LISTEN)、6 機已起飛在空中。
#
# 用法:
#   bash swarm_tot_up.sh                                  # 預設目標/扇形
#   TLAT=25.14 TLON=121.2325 RFAN=2000 HALF=30 bash swarm_tot_up.sh
# 停:Ctrl+C 或 pkill -x tot_node
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash"

TLAT="${TLAT:-25.1406}"; TLON="${TLON:-121.2325}"; TALT="${TALT:-90}"
APPROACH="${APPROACH:-180}"; RFAN="${RFAN:-2000}"; HALF="${HALF:-30}"
VMIN="${VMIN:-14}"; VMAX="${VMAX:-30}"; PLAN="${PLAN:-0.9}"; MARGIN="${MARGIN:-40}"

echo "[tot] 目標=$TLAT,$TLON  扇形 R=$RFAN ±$HALF°  進場自 $APPROACH°"
echo "[tot] ⚠️ 確保編隊節點層已停(pkill -x master_node wingman_node backup_node)"
pkill -x tot_node 2>/dev/null || true
sleep 1
# ROS2 double 參數:整數字面值(如 90)會被判 INTEGER 而拒收 → 一律補小數點
f() { case "$1" in *.*) echo "$1";; *) echo "$1.0";; esac; }
exec ros2 run sar_swarm tot_node --ros-args \
  -p tlat:="$(f "$TLAT")" -p tlon:="$(f "$TLON")" -p talt:="$(f "$TALT")" \
  -p approach_from:="$(f "$APPROACH")" -p r_fan:="$(f "$RFAN")" -p half_angle:="$(f "$HALF")" \
  -p vmin:="$(f "$VMIN")" -p vmax:="$(f "$VMAX")" -p plan_frac:="$(f "$PLAN")" -p margin:="$(f "$MARGIN")"
