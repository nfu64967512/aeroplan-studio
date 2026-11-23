#!/usr/bin/env bash
# swarm_takeoff.sh — 操作員起飛觸發器（配合 MANUAL=1 的 swarm_formation_up.sh）。
#
# 對指定飛機下：TAKEOFF 模式(ArduPlane mode 13) → ARM。
# 飛機以 TAKEOFF 模式自動滾行爬升到 TKOFF_ALT；爬過 guided_min_alt(15m) 後
# sar_swarm 節點會自動把它切回 GUIDED 接手編隊（僚機追 slot、長機帶隊/巡邏）。
#
# 用法：
#   bash swarm_takeoff.sh              # 全部 1..6 依序起飛（間隔 STAGGER 秒）
#   bash swarm_takeoff.sh 1            # 只起飛 UAV1（長機）
#   bash swarm_takeoff.sh 2 3 4        # 起飛指定幾台
#   STAGGER=8 bash swarm_takeoff.sh    # 改起飛間隔（預設 5 秒；跑道縱列防撞）
#
# 前置：swarm_formation_up.sh 已在跑（MAVROS /uav{n}/mavros/* 已上線）。
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash"

STAGGER="${STAGGER:-5}"
UAVS=("$@")
[ ${#UAVS[@]} -eq 0 ] && UAVS=(1 2 3 4 5 6)

echo "[takeoff] 目標: UAV${UAVS[*]}  (間隔 ${STAGGER}s)"
for n in "${UAVS[@]}"; do
  # 1) 切 TAKEOFF 模式（ArduPlane 定翼滾行起飛只吃 mode 13，GUIDED NAV_TAKEOFF 會被拒）
  mode_ok=$(timeout 6 ros2 service call "/uav$n/mavros/set_mode" mavros_msgs/srv/SetMode \
      "{custom_mode: '13'}" 2>/dev/null | grep -c "mode_sent=True\|mode_sent: true" || true)
  # 2) ARM → TAKEOFF 模式下解鎖即自動全油門滾行爬升
  arm_ok=$(timeout 6 ros2 service call "/uav$n/mavros/cmd/arming" mavros_msgs/srv/CommandBool \
      "{value: true}" 2>/dev/null | grep -c "success=True\|success: true" || true)
  if [ "${mode_ok:-0}" -ge 1 ] && [ "${arm_ok:-0}" -ge 1 ]; then
    echo "[takeoff] UAV$n: TAKEOFF + ARMED ✓"
  else
    echo "[takeoff] UAV$n: 失敗 (mode_ok=$mode_ok arm_ok=$arm_ok) — 檢查 /uav$n/mavros 是否上線"
  fi
  # 最後一台不用等
  [ "$n" != "${UAVS[-1]}" ] && sleep "$STAGGER"
done
echo "[takeoff] 完成。爬過 15m 後節點自動接手 GUIDED 編隊。"
echo "[takeoff] 監看：ros2 topic echo /swarm/heartbeat   （或 AeroPlan HUD）"
