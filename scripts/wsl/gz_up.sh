#!/usr/bin/env bash
# gz_up.sh — 起 Gazebo(Harmonic / gz-sim 8)並掛上 ardupilot_gazebo plugin。
#
# ⚠️⚠️ **GZ_SIM_* 兩個路徑一定要在這裡設。**
#    它們只寫在 ~/.bashrc,而 .bashrc 在**非互動 shell 不會被讀**
#    ⇒ 從腳本、cron、或 `wsl.exe -- bash script.sh` 起動時兩個變數都是空的
#    ⇒ plugin 載不到、model:// 找不到,而 gz 的錯誤訊息**不會直指這個原因**
#      (只會說找不到 model 或 system plugin)。這是本專案最愛踩的那類靜默失敗。
#
# 用法:
#   bash gz_up.sh                    # GUI(需要 WSLg)
#   HEADLESS=1 bash gz_up.sh         # 只起 server,用來驗 SDF 與 plugin 能不能載
#   WORLD=/path/to/x.sdf bash gz_up.sh
#
# ⚠️ 起 Gazebo 前先把 SITL 堆疊收乾淨(swarm_sitl_down.sh / swarm_ros2_down.sh):
#    ardupilot_gazebo 的 SITL 要用 `--model gazebo-*` 另外起,和純 SITL 的
#    `--model plane-jet` 是**互斥**的兩種後端,同時跑會搶同一批埠。
set -o pipefail

AG="${AG:-$HOME/ardupilot_gazebo}"
SR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${WORLD:-$SR/gazebo/taoyuan_runway.sdf}"

if [ ! -f "$AG/build/libArduPilotPlugin.so" ]; then
  echo "[gz] ✗ 找不到 $AG/build/libArduPilotPlugin.so —— ardupilot_gazebo 沒編好?" >&2
  exit 1
fi
if [ ! -f "$WORLD" ]; then
  echo "[gz] ✗ 找不到世界檔 $WORLD" >&2
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
# GZ_EXTRA_RESOURCE:gz_gen_world.py 產生的逐機模型放在 repo 外面
# (每機一份、只有 fdm_port_in 不同),要掛進來才找得到。
export GZ_SIM_RESOURCE_PATH="${GZ_EXTRA_RESOURCE:+$GZ_EXTRA_RESOURCE:}$AG/models:$AG/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "[gz] 世界檔      : $WORLD"
echo "[gz] plugin path : $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[gz] resource    : $GZ_SIM_RESOURCE_PATH"

# 先驗 XML 是否 well-formed。打錯標籤是最常見也最容易被淹掉的錯,
# gz sim 會把它混在一堆 warning 裡。
#
# ⚠️ **不要用 `gz sdf -k` 做這件事。** 它沒有呼叫 `sdf::setFindCallback()`,
#    所以**無法解析 `model://`**,對任何有 <include> 的世界檔都會回
#        Unable to find uri[model://runway]
#    ⇒ 世界檔完全正確也一定「失敗」。實測 2026-08-11 撞過。
#    model:// 能不能解析,只有真的起 gz sim 才算數(靠 GZ_SIM_RESOURCE_PATH)。
if ! python3 -c "import sys,xml.etree.ElementTree as ET; ET.parse(sys.argv[1])" "$WORLD" 2>/tmp/gz_xml.log; then
  echo "[gz] ✗ 世界檔 XML 格式錯誤:" >&2
  cat /tmp/gz_xml.log >&2
  exit 1
fi
echo "[gz] ✔ XML well-formed(model:// 的解析交給 gz sim)"

if [ "${HEADLESS:-0}" = "1" ]; then
  echo "[gz] 起 server(headless,-s -r)…"
  exec gz sim -s -r -v 2 "$WORLD"
else
  echo "[gz] 起 GUI + server(-r)…"
  exec gz sim -r -v 2 "$WORLD"
fi
