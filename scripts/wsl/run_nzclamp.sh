#!/usr/bin/env bash
# run_nzclamp.sh — 受控對照:LOS 地板之後**再夾一次** n_z(`nz_clamp_after_floor`)。
#
# ⚠️⚠️ 這個檔存在的理由,就是 [[two1-config-not-in-repo]] 那條教訓:
#    RRE 的達標組態**只活在一次命令列**。這裡把它釘進檔案。
#
# ⚠️⚠️ 第一版本檔包成 run_west.sh,**是錯的**,已在起飛階段攔下。
#    RRE 走的是 **rep_interactive.sh(互動/右鍵打擊路徑)**,證據三條:
#      1. runs/artifacts/RRE1/tot_RRE1_geom.csv 的 target
#         = (25.1261649, 120.615648) = rep_interactive.sh:40-41 的預設,
#         而 run_west.sh 的預設是 (25.0777, 119.94315) —— **差 68 km**;
#      2. ~/fixwing_ws/runs/ 有 RRE{1..8}_pub.log / _contract.log
#         —— 只有 rep_interactive.sh 會產這兩種檔;
#      3. RRE_summary.txt 的檔名形狀 = rep_interactive.sh:287。
#    兩條路徑**不是同一架飛機**:rep_interactive.sh 刻意 `unset JET_THRMAX`
#    ⇒ THR_MAX 留在 100;run_west.sh 走 JET_THRMAX=25 + 爬升段動態提升。
#    見 [[jet-config-not-mirrored]]、[[thrmax-needs-climb-boost]]。
#
# ── 為什麼要跑這個 ────────────────────────────────────────────────────
# 32 架次逐拍重放(matlab/gammadiag/n1_nzmin.py):
#
#   現行(地板繞過夾制)  cmd_nz 最低 **−18.28 g**,29.3% 的拍低於 −2.0 g
#   而契約表宣告的範圍是  pn_nz_min ∈ [−2.0, 0.0]
#
#   ⇒ **我們一直在契約外飛。** 契約層查的是「參數值」,查不到「送出去的指令」。
#
# 那些指令集中在**俯衝入場的推頭段**(frac<0.25 佔 90.91%),不是終端假象。
#
# ⚠️ 但重放同時否掉一個好聽的故事:改動幅度與觸地誤差的
#    spearman = **+0.126(n=32)** ⇒ **不能**說這是打短的成因。
#    判準與功效分析寫在 docs/nzc_decision_rule.md,**資料到達前**定稿。
#
# ── 對照組不用飛 ──────────────────────────────────────────────────────
# matlab/gammadiag/n1_identity.py:工作區的律與 RRE 當時的律(a4ddd55)
# 在 24102/24102 拍**逐位元相同**(roll/pitch/thr 最大差 0)
# ⇒ 已飛完的 RRE(15/16、觸地誤差中位 0.80 m)就是合法對照組。
#
# 用法:
#   bash run_nzclamp.sh                                          # 處理組 NZC,8 趟
#   NZ_CLAMP_AFTER_FLOOR=False PREFIX=NZB bash run_nzclamp.sh     # 重飛對照(如需)
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"

# ── RRE 的三顆開關(rep_interactive.sh 裡沒有,swarm_ros2_up.sh 預設 False)──
# 來源:~/fixwing_ws/runs/RRE1_contract.log 的讀回值,三顆都是 1.0。
export RELEASE_AT_RING="${RELEASE_AT_RING:-True}"
export WAIT_GOTO_RADIUS="${WAIT_GOTO_RADIUS:-True}"
export DIVE_FROM_RUNUP="${DIVE_FROM_RUNUP:-True}"

# 其餘沿用 rep_interactive.sh 預設,契約閘門會逐項讀回比對:
#   目標 (25.1261649, 120.6156480) / approach 94.8 / amsl 0.0(海面)
#   JET=1 / JET_CDP 0.01642 / JET_THRMAX **不設**(THR_MAX=100)
#   gate_r 4700 / perch_alt 3100 / dive_angle 27 / half_angle 25
#   pn_los_floor 1.0 / N=2
export N="${N:-2}"                          # RRE 是 2 機 × 8 趟 = 16 架次

# ── 本次唯一的自變數 ──────────────────────────────────────────────────
# ⚠️ 單變數紀律:pn_nz_min 維持 −1.0(RRE 值)。放寬到 −2.0 是**下一個**實驗。
export NZ_CLAMP_AFTER_FLOOR="${NZ_CLAMP_AFTER_FLOOR:-True}"
export PN_NZ_MIN="${PN_NZ_MIN:--1.0}"

export PREFIX="${PREFIX:-NZC}"
export TRIALS="${TRIALS:-8}"

echo "[nzclamp] 路徑 = rep_interactive.sh(與 RRE 同一條)"
echo "[nzclamp] 自變數 NZ_CLAMP_AFTER_FLOOR=${NZ_CLAMP_AFTER_FLOOR}" \
     "/ PN_NZ_MIN=${PN_NZ_MIN}"
echo "[nzclamp] 對照組 = 已飛的 RRE(15/16、觸地誤差中位 0.80 m);" \
     "逐位元同律已由 n1_identity.py 證實(24102/24102 拍)"
echo "[nzclamp] ${N} 機 × ${TRIALS} 趟 = $((N*TRIALS)) 架次,PREFIX=${PREFIX}"

exec bash "$SR/rep_interactive.sh"
