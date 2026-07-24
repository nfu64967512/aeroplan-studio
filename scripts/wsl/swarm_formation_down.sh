#!/usr/bin/env bash
# swarm_formation_down.sh — 停止 swarm_formation_up.sh 拉起的整個堆疊。
# 用 pkill -x（比對程式名而非命令列）+ bracket regex，避免 pkill -f 自殺
# （命令列含 "master_node" 等字串時 pkill -f 會殺到自己的 shell）。
pkill -x arduplane   2>/dev/null && echo "[down] arduplane 已停"
pkill -x mavros_node 2>/dev/null && echo "[down] mavros 已停"
for p in "[m]avproxy.py" "[m]aster_node" "[w]ingman_node" "[b]ackup_node" \
         "[a]pm_sitl" "[t]ail -f /dev/null"; do
  pkill -f "$p" 2>/dev/null
done
sleep 1
echo "[down] 殘留: arduplane=$(pgrep -xc arduplane 2>/dev/null || echo 0) \
mavros=$(pgrep -xc mavros_node 2>/dev/null || echo 0) \
mavproxy=$(pgrep -fc '[m]avproxy' 2>/dev/null || echo 0) \
nodes=$(pgrep -fc '[_]node' 2>/dev/null || echo 0)"
