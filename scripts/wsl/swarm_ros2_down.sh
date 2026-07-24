#!/usr/bin/env bash
# swarm_ros2_down.sh — 只停 ROS2 層（MAVROS + 長僚機節點）；SITL / MAVProxy 不動。
# 停完可直接再跑 swarm_ros2_up.sh 重新接管（飛機還在飛也沒關係）。
pkill -x mavros_node 2>/dev/null && echo "[ros2-down] mavros 已停"
for p in "[m]aster_node" "[w]ingman_node" "[b]ackup_node" "[a]pm_sitl"; do
  pkill -f "$p" 2>/dev/null
done
sleep 1
echo "[ros2-down] 殘留: mavros=$(pgrep -xc mavros_node 2>/dev/null || echo 0) nodes=$(pgrep -fc '[_]node' 2>/dev/null || echo 0)"
echo "[ros2-down] SITL 仍在: arduplane=$(pgrep -xc arduplane 2>/dev/null || echo 0) mavproxy=$(pgrep -fc '[m]avproxy' 2>/dev/null || echo 0)"
