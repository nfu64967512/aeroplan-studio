#!/usr/bin/env bash
# repro_two1.sh — 重現 TWO1(2026-08-06)那一趟的組態。
#
# ⚠️⚠️ 為什麼需要這支:
#   TWO1 是目前**唯一終端達標**的航次(2/2 命中、觸地誤差 1.95 / 4.04 m、
#   殘餘高度 0.72 / 2.59 m、觸地地速 260 m/s),而它的組態**不存在於任何檔案裡** ——
#   它是 run_west.sh 的預設 + stot_trial.sh 的預設 + **只活在某次命令列的五項覆寫**。
#   repo 與 WSL 都查不到那行命令。這正是 run_west.sh 檔頭自己警告過的失敗形態
#   (「這支先前只存在於臨時 shell…下一輪接手時整組幾何參數就消失了」),TWO1 又重演一次。
#
# 證據來源(WSL:~/fixwing_ws/runs/trial_TWO1.log,節點自報的生效值):
#   :1  auto_ring:perch 3100m / 角度 27° → pushover_r 480 → 6084m
#   :2  扇形 R=35000 ±5° 2 slot 間隔 6109m → 目標 (25.0777, 120.73664)
#   :3  reach=2400 perch_reach=260 impact_reach=25 vmin=95 vmax=100 margin=40
#       layer_m=25 dive_radius=5 hold_radius=400 n=2
#   :11 依實測地形重算:紅心地表 z_g=-32m ⇒ 真實落差 3132m;環半徑 6084 → 6147m
#   :496/:497 UAV2 交接終端 PN @ d=4793m、UAV1 @ d=4797m [gate_r 觸發] ⇒ gate_r=4800
#   ⇒ 真正生效的 gate/環半徑比 = 4800 / 6147 = **0.781**(不是 0.789)
#
# ⚠️ stot_fast.env **沒有**參與 TWO1。該檔的 REACH=1600 / IMPACT_REACH=70 / VMIN=70 /
#    VMAX=155 / MARGIN=60 / DIVE_RADIUS=30 與上面逐項對不上。文件裡「stot_fast.env 是
#    唯一參數來源」的說法與實測不符,不要據此推論現況。
#
# ⚠️ 本檔刻意只設「命令列那五項」,其餘一律沿用 run_west.sh 的預設 —— 因為那些預設
#    正是 TWO1 當時吃到的值。不要把 stot_fast.env source 進來:該檔用**裸賦值**,
#    會全面壓過 run_west.sh 的 ${VAR:-},連 JET_CDP / REACH / VMIN 一起換掉。
#
# 用法:
#   bash repro_two1.sh                      # 單趟,PREFIX=TWO
#   PREFIX=NF TRIALS=3 bash repro_two1.sh   # 三趟,建立雜訊底線(兩機的目前未知)
#   KILL_ON_TOUCHDOWN=False bash repro_two1.sh   # 關掉觸水終結做 A/B
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"

# ── TWO1 的五項命令列覆寫(全部可再從命令列覆寫)──────────────────────
export DIVE_ANGLE="${DIVE_ANGLE:-27.0}"     # run_west.sh 預設是 45.0
export GATE_R="${GATE_R:-4800.0}"           # run_west.sh 不設此項 ⇒ 會落到
                                            # stot_trial.sh:23 的預設 1200.0
export HALF_ANGLE="${HALF_ANGLE:-5.0}"      # run_west.sh 預設是 25.0
export TGT_LON="${TGT_LON:-120.73664}"      # run_west.sh 預設是 119.94315
export SWARM_COUNT="${SWARM_COUNT:-2}"      # 兩機起步

export PREFIX="${PREFIX:-TWO}"
export TRIALS="${TRIALS:-1}"

echo "[repro_two1] 重現 TWO1 組態:俯角 ${DIVE_ANGLE}° / GATE_R ${GATE_R} /" \
     "半角 ±${HALF_ANGLE}° / ${SWARM_COUNT} 機 / 目標經度 ${TGT_LON}"
echo "[repro_two1] 環半徑 = (perch − dive_alt)/tan(俯角);地形重算後 6147 m" \
     "⇒ gate/環 ≈ $(awk -v g="$GATE_R" 'BEGIN{printf "%.3f", g/6147}')"
echo "[repro_two1] 其餘一律沿用 run_west.sh 預設(vmin 95 / reach 2400 /" \
     "JET_CDP 0.01642 / TARGET_AMSL 0.0)"

exec bash "$SR/run_west.sh"
