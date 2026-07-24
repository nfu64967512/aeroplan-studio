#!/usr/bin/env bash
# swarm_sitl_down.sh — 只停 SITL 層（arduplane + mavproxy 分流）。
# 若 ROS2 層還在跑，MAVROS 會斷線重連等待；通常先 swarm_ros2_down.sh 再停這層。
pkill -x arduplane 2>/dev/null && echo "[sitl-down] arduplane 已停"
pkill -f "[m]avproxy.py" 2>/dev/null
pkill -f "[t]ail -f /dev/null" 2>/dev/null
sleep 1
echo "[sitl-down] 殘留: arduplane=$(pgrep -xc arduplane 2>/dev/null || echo 0) mavproxy=$(pgrep -fc '[m]avproxy' 2>/dev/null || echo 0)"
