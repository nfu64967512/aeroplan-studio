#!/usr/bin/env bash
# swarm_ros2_up.sh — 第 2 層：6×MAVROS + 長僚機（master/wingman/backup）ROS2 launch
#                    + tot_node（終端導引，STANDBY 待命）。
#
# 與 swarm_sitl_up.sh 成對使用（SITL / ROS2 控制分離）：
#   前置：swarm_sitl_up.sh 已在跑（分流埠 5860+10i 已 LISTEN）。
#   本層可隨時 swarm_ros2_down.sh 停掉再重開 —— SITL 不受影響。
#
# ══ 2026-08-05：tot_node 一併拉起（STANDBY）═══════════════════════════════
# 目的是「兩個 CMD 就能互動打擊」：
#   CMD1  bash swarm_sitl_up.sh    → 6 架 SITL（AeroPlan 連 6760+10i 就看得到）
#   CMD2  bash swarm_ros2_up.sh    → MAVROS×6 + master/backup/wingman + tot_node
#   之後在 AeroPlan 地圖右鍵指定紅心 → 送 /swarm/tot_target → tot_node 開打
#
# ⚠️⚠️ 本層與**批次航次**（run_west.sh → stot_repeat.sh → stot_trial.sh）是
#    兩條不同用途的路徑，不要混用：
#      批次：每趟開頭 swarm_sitl_down + up + sleep 150（會拆掉並重建 SITL）
#      本層：常駐，**絕不動 SITL**
#    兩條路徑各自會起一個 tot_node，同時活著就會互相蓋 DO_REPOSITION 而且
#    完全靜默 —— 所以底下的前置檢查會把它們分辨出來（見「殘留的 tot_node」段）。
#
# tot_node 的幾何參數：預設值**逐條抄自 stot_trial.sh**（該檔是參數真相源，
# 每一行都有實測血淚註解），並照本腳本慣例用 ${VAR:-預設} 開放 env 覆寫。
# 手動指定目標（不透過 AeroPlan）：
#   ros2 topic pub --once /swarm/tot_target std_msgs/String '{data: "25.1406,121.2325"}'
#
# origin / launch_type / 台數 自動讀 ~/fixwing_ws/sitl_run/swarm_state.env
# （swarm_sitl_up.sh 寫入），保證編隊原點 = SITL 生成點（不會順移）。
# 也可用 env 顯式覆寫（LOC= / LAUNCH= / COUNT=）。
#
# 用法：
#   bash swarm_ros2_up.sh                       # 待命：不 arm 不起飛（預設 MANUAL=1）
#   MANUAL=0 bash swarm_ros2_up.sh              # 全自動 arm+起飛+巡邏
#   PATROL=0 bash swarm_ros2_up.sh              # 你起飛後長機定點（不巡邏），僚機仍編隊
#   FORMATION=biomimetic bash swarm_ros2_up.sh  # 仿生 V
#   WAIT_SPEED=6 FORMED_RADIUS=120 CATCH_SPEED=25 ...   # 編隊位置約束調校
#
# 之後起飛由你觸發：bash swarm_takeoff.sh [n...]
# 停止本層：bash swarm_ros2_down.sh（或 Ctrl+C）
set -eo pipefail

BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
STATE="$BASE/swarm_state.env"

# ── 讀 SITL 層狀態檔（可被 env 覆寫）──────────────────────────────
SWARM_LOC=""; SWARM_LAUNCH=""; SWARM_N=""
# shellcheck disable=SC1090
[ -f "$STATE" ] && source "$STATE"
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    TAOYUAN|RCTP|TPE) echo "25.0777,121.2325,32,52" ;;
    HUWEI)      echo "23.702732,120.419333,0,0"  ;;
    SWARM)      echo "22.60,120.00,0,90" ;;
    "")         echo "" ;;
    *)          echo "$1" ;;
  esac
}
LOC="$(resolve_loc "${LOC:-}")"
LOC="${LOC:-${SWARM_LOC:-25.0777,121.2325,32,52}}"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"
LT="${LAUNCH:-${SWARM_LAUNCH:-runway}}"
N="${COUNT:-${SWARM_N:-6}}"

case "$(echo "${FORMATION:-v}" | tr '[:upper:]' '[:lower:]')" in
  biomimetic|bio|swarm) BIO=true ;;
  *)                    BIO=false ;;
esac

# MANUAL=1（預設）：節點不 arm 不起飛，起飛由 swarm_takeoff.sh / GCS 觸發；
# 爬過 15m 後節點自動接手 GUIDED 編隊。PATROL 獨立（預設 1 = 起飛後長機帶隊巡邏）。
MANUAL="${MANUAL:-1}"
if [ "$MANUAL" = "1" ]; then AA=false; AT=false; else AA=true; AT=true; fi
PATROL="${PATROL:-1}"
[ "$PATROL" = "1" ] && AP_PATROL=true || AP_PATROL=false
RAFT_ARG="${RAFT:-0}"; [ "$RAFT_ARG" = "1" ] && RAFT_ARG=true || RAFT_ARG=false

# ══ PRESET：一鍵套用「收斂 vs 速度」兩組實測調參 ═══════════════════════════
# 實測結論（收斂與速度是物理取捨，非調參 bug）：
#   固定翼轉彎半徑 r≈v²/(g·tanφ)。要把僚機「收」進 slot，catch 速度的轉彎半徑
#   必須 < 編隊間隔；速度越高半徑越大 → 高速下僚機只能繞著 slot 打轉收不進來。
#   另外長機要能「慢下來等」才有速度差讓僚機追上。
#     • tight（泡棉 plane 模型，min~10 / max~30 m/s）：長機可慢到 12、僚機追到 30，
#       速度差大、轉彎半徑小 → 實測 worst slot err 收斂到 ~50m（直線）~170m（轉角），
#       全程編隊成形、長機會自動 12↔22 手風琴等待。但巡航僅 ~22 m/s。
#     • fast（噴射 plane-jet，實測最低可持平飛 ~45 / max~76 m/s）：速度快，但最低
#       平飛速度就給出 ~500m 轉彎半徑（比任何合理間隔都大），物理上收不緊 →
#       實測只能維持 ~300m 內的凝聚雲團（不脫隊、不亂飛，但非緊 V）。
#   ⇒ 想「緊 V」用 tight（配 SITL：不要 JET）；想「最快」用 fast（配 SITL：JET=1）。
# 用法：PRESET=tight bash swarm_ros2_up.sh  /  PRESET=fast bash swarm_ros2_up.sh
case "$(echo "${PRESET:-}" | tr '[:upper:]' '[:lower:]')" in
  tight|foam|slow)
    : "${NOMINAL_SPEED:=22.0}" "${WAIT_SPEED:=12.0}" "${CATCH_SPEED:=30.0}"
    : "${CATCH_RADIUS:=500.0}" "${FORMED_RADIUS:=50.0}"
    : "${V_SPACING:=60.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=120.0}"
    : "${PATROL_BOX:=1500.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=6.0}"
    echo "[ros2] PRESET=tight（緊 V，需 SITL 用泡棉 plane 模型／不要 JET=1）" ;;
  fast|jet)
    : "${NOMINAL_SPEED:=35.0}" "${WAIT_SPEED:=45.0}" "${CATCH_SPEED:=76.0}"
    : "${CATCH_RADIUS:=900.0}" "${FORMED_RADIUS:=150.0}"
    : "${V_SPACING:=120.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=220.0}"
    : "${PATROL_BOX:=4000.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=10.0}"
    echo "[ros2] PRESET=fast（最快 ~76 m/s，凝聚雲團非緊 V；需 SITL JET=1）" ;;
  max|plana|maxv)
    # Plan A 實測冠軍（2026-07-23 iter7）：泡棉機頂到氣動天花板還保住 V。
    # 三個關鍵機制（缺一不可，前兩個是程式修改）：
    #   1. speed_for_error 斜坡從 err=0 起算（否則 err 漸近卡在 formed_r 上緣永不釋放）
    #   2. wp_lead_time slot 前饋（WP 瞄外推點抵消 ~3.5s 導引滯後；誤差仍對真 slot 量）
    #   3. formed_hysteresis 1.8（巡航誤差膨脹 ~+70m，1.4 會一釋放就被打回 wait）
    # 實測：長機平均 25.0 m/s、巡航佔比 85%、誤差 95(直線)~173(轉角) 有界手風琴、
    # 零脫隊。需 SITL 泡棉 + MAXV=1（AIRSPEED_MAX 35 解 30 參數上限；氣動極限 ~31.5）。
    : "${NOMINAL_SPEED:=27.0}" "${WAIT_SPEED:=16.0}" "${CATCH_SPEED:=31.0}"
    : "${CATCH_RADIUS:=300.0}" "${FORMED_RADIUS:=95.0}" "${FORMED_HYST:=1.8}"
    : "${WP_LEAD:=3.5}"
    : "${V_SPACING:=60.0}" "${V_ANGLE:=25.0}" "${V_TAIL:=120.0}"
    : "${PATROL_BOX:=2500.0}" "${WP_INTERVAL:=1.0}" "${WP_MOVE_THRESH:=6.0}"
    echo "[ros2] PRESET=max（Plan A：巡航 ~25-27 m/s + V 隊形；SITL 用泡棉 + MAXV=1）" ;;
esac

# 編隊位置約束（長機等僚機 / 僚機追上）調校旋鈕
NOMINAL_SPEED="${NOMINAL_SPEED:-14.0}"
WAIT_SPEED="${WAIT_SPEED:-10.0}"
FORMED_RADIUS="${FORMED_RADIUS:-250.0}"
CATCH_SPEED="${CATCH_SPEED:-20.0}"
CATCH_RADIUS="${CATCH_RADIUS:-400.0}"
# V 隊形幾何（間隔）：V_SPACING=編隊間隔 m（越小越密）；V_ANGLE=V 半角；V_TAIL=備長跟隨距離
# ⚠️ 要收緊到 ~50m：V_SPACING=50 且 FORMED_RADIUS 要 < 50（例 30），否則永遠算「到位」不收緊。
V_SPACING="${V_SPACING:-120.0}"
V_ANGLE="${V_ANGLE:-25.0}"
V_TAIL="${V_TAIL:-220.0}"
# WP 追蹤：導引航點刷新間隔 / slot 移動門檻。越小＝slot 航點越連續 → 少 S 形蛇行。
# 預設已收緊到 1.0/10（節點內建 3.0/25 對移動編隊太粗會蛇行）。
WP_INTERVAL="${WP_INTERVAL:-1.0}"
WP_MOVE_THRESH="${WP_MOVE_THRESH:-10.0}"
# 長機/編隊巡航高度帶（origin altitude 參數）。⚠️ 不可用舊 sim 值 1000，否則長機
# 會爬到 1000m 遠離編隊(patrol_alt≈80m)看似不收斂。預設 100 與編隊同高度帶。
ORIGIN_ALT="${ORIGIN_ALT:-100.0}"
# 巡邏方框邊長 m。高速下加大（直線段長、轉角少）→ 僚機有足夠直線收斂 slot。
PATROL_BOX="${PATROL_BOX:-1500.0}"
# 成形解除門檻倍數（master formed_hysteresis）。巡航中 slot 誤差比等待時膨脹 ~+70m,
# 1.4 太窄會「一釋放就被打回 wait」群速卡在 wait_speed;拉高 nominal 時建議 ~1.8。
FORMED_HYST="${FORMED_HYST:-1.4}"
# slot 前饋秒數:目標 = slot 沿長機航向外推 lead×leader_speed,抵消導引 WP ~3-5s
# 有效滯後(高速巡航誤差地板 ≈ v×lag,27m/s 時 ~100-150m)。0=關;高速編隊建議 3-4。
WP_LEAD="${WP_LEAD:-0.0}"
# §4.5 轉彎程序:弧轉半徑(slot 掃掠率=wait_speed/R;大=轉角僚機跟得上)+ 轉前減速距離
TURN_RADIUS="${TURN_RADIUS:-800.0}"
PRESLOW_MARGIN="${PRESLOW_MARGIN:-250.0}"

# ══ tot_node（終端導引，STANDBY 待命）的參數 ═══════════════════════════════
# ⚠️⚠️ **每一個值都是抄 stot_trial.sh 的既有預設，不是自己編的。**
#    抄一份到這裡而不是 source 那支腳本，是因為 stot_trial.sh 一被 source 就會
#    開始跑航次（它不是純參數檔）。代價是兩邊會漂移 —— 所以每一條都標了來源
#    行號，tests/test_tot_standby.py 有一條迴歸鎖去比對兩邊的數值。
# ⚠️ ROS 2 的 -p 會把 "200" 判成 INTEGER，而參數宣告是 DOUBLE ⇒ 節點啟動即崩
#    （InvalidParameterTypeException）。所有 float 一律補小數點 —— 這個陷阱
#    stot_trial.sh:92 已經踩過兩次，照抄它的 _f()，不要靠呼叫端記得寫 400.0。
_f() { case "$1" in *.*|*e*|*E*) printf "%s" "$1";; *) printf "%s.0" "$1";; esac; }
TOT_ENABLE="${TOT_ENABLE:-1}"                       # 0 = 不起 tot_node
TOT_OUT_DIR="${TOT_OUT_DIR:-$HOME/fixwing_ws/runs}"
TOT_CSV="${TOT_CSV:-$TOT_OUT_DIR/tot_standby}"      # 第 2 發起自動變 _s2/_s3…
TOT_LOG="${TOT_LOG:-$TOT_OUT_DIR/tot_standby.log}"
# ⚠️⚠️ 分流埠：tot_node 走 **6860+10i**（它專屬的那一路）。
#    MAVProxy 的 `--out=tcpin` 只餵「先連上的那個 client」，共用同一個埠時
#    後連的那邊會收不到 GLOBAL_POSITION_INT，而且完全沒有錯誤訊息。
#
#    2026-08-07 之前這裡是 6960 —— 那是**錯的**，因為 6960+10i 同時是
#    編隊層的 WP 鏈路：master_node 連 6960、backup_node 連 6970，而且它們
#    在 launch 檔裡比 tot_node 早起。待命 tot_node 永遠是第二 client
#    ⇒ 收到 0 bytes ⇒ `ready_for_target` 恆 false ⇒ AeroPlan 右鍵打擊
#    按下去毫無反應。實測位元組計數與完整推導見 swarm_sitl_up.sh 的
#    `_launch_bridge` 註解。
#
#    ⚠️ 批次路徑（stot_trial.sh）不受影響也不要跟著改：它在交棒前會
#    `swarm_ros2_down.sh` 收掉編隊層，6960 是空的，而 6960 那組是
#    TWO1 唯一達標組態驗證過的埠。
TOT_LINK_BASE="${TOT_LINK_BASE:-6860}"
TOT_LINK_STRIDE="${TOT_LINK_STRIDE:-10}"
# 幾何（來源 stot_trial.sh:62-74、48-49、42-43）
#
# ⚠️⚠️⚠️ 2026-08-07:**這一段的預設值原本鏡像的是 stot_trial.sh 的預設,
#    而批次航次跑的是 run_west.sh 覆蓋後的值。** run_west.sh 覆寫了其中 6 項
#    (VMIN / PERCH_CLIMB_V / WEAVE_PERIOD / PN_VMAX / DIVE_SPEED / AIRSPEED_MAX_CAP),
#    全都是**噴射機改裝時改的**。互動路徑看不到那一層覆蓋,於是忠實地複製了一組
#    **噴射機改裝後沒有任何一趟航次用過**的泡棉機基準。
#    症狀(AP3 實測):HUD 巡航只有 82 m/s,而機體巡航能力是 ~157。
#    下面逐項把 run_west.sh 的生效值搬過來,每一項都附它自己的理由。
# ⚠️⚠️⚠️ **3100 m 噴射機巡航剖面(2026-08-07 使用者定調)**
#    下面六項是**一整組耦合的幾何**，來自 run_west.sh(批次路徑的生效值，
#    也就是 TWO1 唇一達標組態跑的那一組)。**不可以只改其中一項**：
#
#        環半徑 = (perch_alt − dive_alt) / tan(dive_angle)
#          3100 / tan(27°) = **6084 m**      ← 這一組
#          3100 / tan(10°) = 17575 m         ← 只改高度的下場，幾何直接爆掉
#           400 / tan(10°) =  2269 m         ← 舊值(泡棉機尺度)
#
#    為什麼要 3100 m：使用者要的「巡航 157 m/s」是 **3100 m 的地速**
#    (run_west.sh:68：157 m/s 地速 @3100 m = EAS 135)。同樣的 EAS 135 在 400 m
#    只有 ~138 m/s 地速 —— 要在 HUD 上真的看到 157，**必須把巡航高度拉上去**，
#    不是再把 vmax 往上加(機體推不動，加了只會飽和)。
#
#    ⚠️ HALF_ANGLE **刻意留在 25.0**：TWO1 用的是 5.0，但那是 n=2 的特例化；
#      n=6 時扇形收太小會讓 ALIGN 收不完(b500-wide-spacing-status：UAV4 卡死)。
#      本層要對任意機數成立 ⇒ 留寬的；n=2 重現 TWO1 請下 HALF_ANGLE=5。
# 扇形半徑：蜜群在距紅心 35 km 處集結，再向內飛到 6.1 km 環。
TOT_R_FAN="$(_f "${R_FAN:-35000.0}")"
TOT_HALF_ANGLE="$(_f "${HALF_ANGLE:-25.0}")"
# ⚠️ vmin 是 ALIGN/PERCH 的等待速度,也是 **weave 的啟動門檻**(v_req < vmin 才走 S 型)。
#    舊值 45 低於可持續飛行包絡(sep_envelope.py 在 3100 m 量到 SEP 零交越在 EAS 72-73)
#    ⇒ 要求飛機用飛不動的速度耗時間,結果是往下掉。來源 run_west.sh:250-261。
TOT_VMIN="$(_f "${VMIN:-95.0}")"
# ⚠️⚠️ **巡航速度就是這一項決定的。** tot_node 用 `v_plan = plan_frac × vmax`
#    規劃共同抵達時刻(tot_node.py:2059/2491/2582),plan_frac 兩條路都寫死 0.9 ⇒
#        v_plan = 0.9 × vmax
#    舊值 100 ⇒ v_plan = 90 EAS,實測 HUD 巡航只有 82 m/s。
#
# ⚠️⚠️⚠️ **「157 m/s」是地速 @3100 m,不是 EAS。** 這一格我第一次改錯過:
#    run_west.sh:68 原文 ——「157 m/s 地速 @3100 m = **EAS 135**」,
#    而 run_west.sh:72 就是在警告這個坑 ——「--cruise 要餵 EAS;餵 157
#    會設計成 3100 m 的 183」。機體正是照 `--cruise 135` 反推出來的
#    (c_drag_p 0.01642 / THR_MAX 25),所以**設計巡航就是 EAS 135**。
#    我第一版設 175 ⇒ v_plan = 157.5 EAS,比機體天花板高 17% ⇒
#    tot_node 會讓全隊卡在 `mode="max"` 的飽和點,T_arr 永遠達不到,
#    **共同抵達的同步迴路等於變成開迴路**,散布會被凍結在形成當下的值 ——
#    對 STOT 來說這是最糟的一種失敗:看起來在飛,但同步已經沒有作用。
#    ⇒ 150 ⇒ v_plan = **135.0 EAS** = 設計巡航,指令上限 150 EAS 仍留餘裕。
#
#    ⚠️ EAS ≠ 地速,兩者差在空氣密度:
#         3100 m:EAS 135 ⇒ 地速 ≈ 157  ← 使用者要的那個數字
#          400 m:EAS 135 ⇒ 地速 ≈ 138
#      互動路徑的 perch 是 **400 m**,所以 HUD 會顯示 ~138 而不是 157。
#      要在 HUD 上真的看到 157,得把巡航高度拉到 3100 m 一帶(PERCH_ALT),
#      **不是**再把 vmax 往上加 —— 機體推不動,加了只會飽和。
#    ⚠️ 這個值必須 ≤ 飛控的 AIRSPEED_MAX,否則 DO_CHANGE_SPEED **整發被拒**
#      (MAV_RESULT=4,見 catch-speed-rejected-by-fc)。AIRSPEED_MAX 已從
#      165 提到 230(run_west.sh:86 的生效值;jet_design 要求 ≥224)。
#    ⚠️ run_west.sh 沒有覆寫 VMAX,批次路徑仍是 100 —— 因為批次有獨立的
#      巡航段(jet_show_ros2.py)把飛機帶過去,tot_node 只做末端;
#      互動路徑沒有巡航段,ALIGN 要自己飛完 50 km,所以才需要提高。
TOT_VMAX="$(_f "${VMAX:-150.0}")"
TOT_MARGIN="$(_f "${MARGIN:-40.0}")"
# ⚠️ 抵達容差要跟著幾何尺度走。260 是泡棉機尺度；
#    35 km 扇形 + 6 km 環下用 260 會讓 ALIGN 永遠判不到「到了」。來源 run_west.sh:140。
TOT_REACH="$(_f "${REACH:-2400.0}")"
TOT_LAYER_M="$(_f "${LAYER_M:-25.0}")"
TOT_PERCH_REACH="$(_f "${PERCH_REACH:-260.0}")"
# ⚠️⚠️ 守環半徑。**本檔先前完全沒有傳這個參數** ⇒ 互動路徑一路吃 tot_node
#    的宣告預設 400,而 400 < 物理迴轉半徑 1253(vmin 95 EAS @ 3100 m、45°)。
#    節點每次開機都用 ERROR 吼,**沒有任何東西擋下來**,RPK/RPL 九趟全帶著跑。
#    後果:守環退化成繞自己的最小圈 ⇒ 放行時距目標的距離在 ±迴轉半徑 之間隨機
#    (SW1:進環 d 一致 3108–3131,放行卻散到 1806–3449)。
#    ⇒ 極可能就是 handover-altitude-is-the-variable 記的交接高度 ±600 m 散布的來源。
# 1300 = 迴轉半徑 1253 再留餘裕。contract_table 的 `hold_radius ≥ 物理迴轉半徑`
# 不變式會在起飛前驗算,改 vmin / perch_alt 都會重新檢查。
TOT_HOLD_RADIUS="$(_f "${HOLD_RADIUS:-1300.0}")"
TOT_GOTO_RADIUS="$(_f "${GOTO_RADIUS:-30.0}")"
TOT_IMPACT_REACH="$(_f "${IMPACT_REACH:-25.0}")"
TOT_OVERSHOOT="$(_f "${OVERSHOOT:-300.0}")"
TOT_SWITCH_R="$(_f "${SWITCH_R:-300.0}")"
# ⚠️ 俯衝指令空速上限。要**高於機體極限**飛控才會一直壓機頭;
#    140 是泡棉機時代的值。來源 run_west.sh:273。
TOT_DIVE_SPEED="$(_f "${DIVE_SPEED:-210.0}")"
TOT_DIVE_RADIUS="$(_f "${DIVE_RADIUS:-5.0}")"
# 巡航/內切環高度。來源 run_west.sh:143。
TOT_PERCH_ALT="$(_f "${PERCH_ALT:-3100.0}")"
# 扇形集結高度，跟 perch 同高。來源 run_west.sh:144。
TOT_TALT="$(_f "${TALT:-3100.0}")"
# ⚠️ 俯衝角。確定環半徑的就是它，不可以與 perch_alt 分開改。
#    27° 是 stot-refuted-changes 明文「保留下來的改動」之一；
#    硬上限約 60°(dive-angle-ceiling)。來源 run_west.sh:153。
TOT_DIVE_ANGLE="$(_f "${DIVE_ANGLE:-27.0}")"
TOT_DIVE_LAYER="$(_f "${DIVE_LAYER_M:-${DIVE_LAYER:-4.0}}")"
TOT_ALIGN_TOL="$(_f "${ALIGN_TOL:-40.0}")"
TOT_TRACK_FRAC="$(_f "${TRACK_FRAC:-0.8}")"
# ⚠️⚠️ **50.0 是泡棉機時代的常數,已改成 0(自動)。**
#    tot_node 的註解要求「配保守一點(≈vmin)才有餘裕讓近機慢下來等遠機」——
#    當年 vmin=45,pace 50 ≈ vmin,是對的。噴射組態 vmin=95 之後 50 ≪ 95,
#    時刻表變成「比最低可飛速度還慢」,物理上不可能達成:
#        v_req = d / t_rem ≈ 46  <  vmin = 95
#        v = max(vmin, min(dive_speed, v_req)) ⇒ **整段俯衝被夾在 95**
#    後果有兩個(RPJ 三趟實測):
#      ① 同步迴路全程飽和、零權限 ⇒ 兩機都以能飛的速度衝完,早到 25–52 s,
#         早到量的差就是命中散布(RPJ3 = 25.00 s)
#      ② `dive_speed=210` **從來沒生效過** ⇒ 終端只有 115–120 m/s
#    ⇒ 一個泡棉常數同時卡住門檻④ 和終端速度 KPI。
# ⇒ 0 = 自動,取 `min(vmax, 0.8·dive_speed)` = min(150, 168) = **150**,
#    正好落在 [vmin 95, dive_speed 210] 中間 ⇒ 兩個方向都有調節權限。
# ⚠️ 不要往回調小:pace < vmin 就等於沒有同步。tot_node 會對這個條件告警。
TOT_DIVE_PACE="$(_f "${DIVE_PACE:-0}")"
TOT_DIVE_SPDW="$(_f "${DIVE_SPDW:-0.0}")"
TOT_GROUND_ALT="$(_f "${GROUND_ALT:-0.0}")"
TOT_PERCH_BAND="$(_f "${PERCH_BAND:-40.0}")"
TOT_PERCH_SETTLE="$(_f "${PERCH_SETTLE:-25.0}")"
# ⚠️⚠️ 2026-08-14(S7)25.0 → 3000.0,與 run_west.sh:226 對齊。
#    這是 jet-config-not-mirrored 那一整類的又一個實例:互動路徑抄 stot_trial.sh:80
#    的 25.0,批次路徑用 run_west.sh 的 3000.0,兩邊從來沒有人比對過。
#
#    25.0 的後果(run_west.sh:216-218 記載的實測):高度偏離下滑線超過 25 m 就重發
#    航點,而重發會呼叫 set_target_altitude_current() 把高度需求**重錨到當下高度**
#    ⇒ 每 resend_min_dt 秒抹掉一次累積的下降需求。實測離環到交接那 1950 m
#    六機**淨爬升 +54~+175 m**,交接時 h/d 高出下滑線 +1694~+2106 m。
#    3000.0 讓重發條件恆為假 = 真正的單發鎖存,不動程式碼。
#
# ⚠️ 這一項對 Hybrid Trajectory Shaping 是**前置條件**:任何「讓等待段下降」的
#    嘗試,只要重發鋸齒還在,累積的下降需求就會被週期性抹回,量不出效果。
# ⚠️ 不要順手改 RESEND_DT(run_west.sh:221 說明理由:拉長週期只是把鋸齒變粗)。
TOT_LINE_TOL="$(_f "${LINE_TOL:-3000.0}")"
TOT_RESEND_DT="$(_f "${RESEND_DT:-2.0}")"
TOT_HDEM_TC="$(_f "${HDEM_TC:-0.5}")"
TOT_IMPACT_ALT_OK="$(_f "${IMPACT_ALT_OK:-30.0}")"
TOT_IMPACT_SETTLE="$(_f "${IMPACT_SETTLE:-4.0}")"
# ⚠️⚠️ PERCH 爬升空速。0 = 用節點預設 max(vmin*1.45, 65)，而 65 是
#    **舊泡棉機體**校出來的 —— 噴射機在這個速度撐不住高度。
#    EL1 實測:UAV3 在 PERCH 以 65.2 m/s「爬升」，高度反而從 2603 m 一路掉到 519 m，
#    永遠達不到就緒閘門，整趥卡死逾時。來源 run_west.sh:242-248。
TOT_PERCH_CLIMB_V="$(_f "${PERCH_CLIMB_V:-100.0}")"
TOT_DIVE_PITCH_MIN="$(_f "${DIVE_PITCH_MIN:--80.0}")"   # ⚠️ 必須是負值，見 stot_trial.sh:106
TOT_DIVE_PITCH_MAX="$(_f "${DIVE_PITCH_MAX:-45.0}")"
# ⚠️⚠️ 2026-08-14 RPY3:這一欄預設 0.0(= 守門員關閉)是 78.5 s 散布的直接成因。
#    放行鏈當時是:時刻表(被時基污染排到 56 年後,永不觸發)→ 幾何守門員
#    (關閉)→ `perch_settle` 25 s 逾時 ⇒ **不看幾何、全隊照放**。
#    UAV1 因此在「還在內衝、距紅心只剩 3454 m 而高度 3058 m」時被放行,
#    所需俯角 41.5° > 當日實測可達天花板 37–40° ⇒ 幾何上從第一筆就無解。
#    它滾到 −97°(翻過刀鋒進負升力區)外飛到 7024 m、高度洩掉 1900 m,
#    再用 8–9° 淺滑翔飛 90 s 回來 —— 137.8 s vs 另一架 59.4 s。
#    ⭐ 四架次的 (所需俯角, 俯衝耗時):(23.0, 59.4)(23.2, 49.4)(28.4, 61.5)
#       (41.5, **137.8**) —— 前三個落在 50–62 s 帶內,只有跨過錐的那架跳掉。
#       這是**二元事件不是漸進退化**,所以「單趟散布 ±10 s 雜訊」的說法
#       只在錐內成立。
# ⚠️⚠️ **但先不要打開**(2026-08-14 二次分析撤回)。守門員的前提是
#    「等待中 γ_req 單調變陡」,那在 PERCH 段成立,**而它不在 PERCH 段跑** ——
#    它掛在 `elif self.phase == "DIVE"` 裡。RPY3 實測:UAV1 在 phase 翻 DIVE
#    那一刻 γ_req 已經 41.5°,接下來 13 s 在 `wait` 模式外飛,γ_req **下降**
#    到 33.5° 才真的放行。若當時 DIVE_ANGLE_MAX=34,守門員會在 41.5° 立刻
#    觸發、把它放進**比等待更差**的幾何裡 ⇒ 開了反而更糟。
#    ⇒ 正確修法是把同一個 γ_req 檢查**往前掛到 PERCH 的放行判定**,
#      而不是單純把這個值調上去。那是一次獨立的設計變更,要單獨驗。
#    在那之前維持 0.0,並且這一欄現在**在契約表裡看得見**(先前完全沒納管)。
TOT_DIVE_ANGLE_MAX="$(_f "${DIVE_ANGLE_MAX:-0.0}")"
TOT_DIVE_THR_MAX="$(_f "${DIVE_THR_MAX:-100.0}")"
TOT_PERCH_STEP="$(_f "${PERCH_STEP:-0.0}")"
TOT_STAGGER_LEAD="$(_f "${STAGGER_LEAD:-0.0}")"
# ⚠️⚠️ 換向週期。22 s 是**泡棉機時代**的:泡棉 14 m/s 轉彎率 40.1°/s，
#    θ=53° 只要 2.6 s ≪ 半週期 11 s。本機體 157 m/s 只有 3.58°/s，
#    把航向從 +θ 甩到 −θ 要 2θ/ω：θ=45° 需 25.1 s > 半週期 11 s
#    ⇒ **θ > 19.7° 根本轉不完就被要求反向**。週期要 ≥ 4× 換向時間。
#    來源 run_west.sh:263-271。
TOT_WEAVE_PERIOD="$(_f "${WEAVE_PERIOD:-120.0}")"
# ⚠️⚠️ 2026-08-14:0.0 → 1.0,與 run_west.sh:238 對齊。
#    這是 jet-config-not-mirrored 那一類裡代價最大的一個:
#      run_west.sh(TWO1 唯一達標航次)   PN_LOS_FLOOR = 1.0
#      swarm_ros2_up.sh(互動/產品路徑)  PN_LOS_FLOOR = **0.0**
#    ⇒ RPK/RPL/RPO/RPQ **二十四架次全部在關閉狀態下飛**。
#
#    0.0 = 純 PN:縱向通道只對視線角速率積分,`dive_angle` 完全不被引用,
#    終端航跡角是積出來的**副產品**而不是被要求的量。
#    tot_node.py 自述:「這是陡俯角**唯一有效的開關**」。
#
#    代價已量化(用校準過的預測器反解 RPQ 六架次):
#      互動路徑實得俯角 **1.0–30.1°,中位 11.5°**
#      批次路徑實測      **23.3–33.9°**
#    其中一架(RPQ3/2)等效俯角只有 1.0° —— 實質上是平飛通過紅心上空。
#
# ⚠️ 必須與 PN_NZ_MIN 一起改:前者要求陡角,後者決定推得下去與否。
TOT_PN_LOS_FLOOR="$(_f "${PN_LOS_FLOOR:-1.0}")"
# ⚠️⚠️ 2026-08-14:0.05 → -1.0,與 run_west.sh:240 對齊。
#    0.05 = 不准負 g 推頭 ⇒ **俯角上限被法向過載下限鎖死**,不是被機體鎖死。
#    tot_node.py:464 自述:「推到 −80° 需要 9259 m 高度,我們只有 3148」。
#    要陡俯角就必須允許小負 g;run_west.sh 用 −1.0。
TOT_PN_NZ_MIN="$(_f "${PN_NZ_MIN:--1.0}")"
TOT_PN_THR_TRIM="$(_f "${PN_THR_TRIM:-0.36}")"
TOT_PN_THR_GAIN="$(_f "${PN_THR_GAIN:-0.030}")"
# miss 出口（來源 stot_trial.sh:124-128）
TOT_MISS_FAR_M="$(_f "${MISS_FAR_M:-20000.0}")"
TOT_MISS_FAR_K="$(_f "${MISS_FAR_K:-2.0}")"
TOT_MISS_WINDOW_S="$(_f "${MISS_WINDOW_S:-60.0}")"
TOT_MISS_SLOW_GS="$(_f "${MISS_SLOW_GS:-5.0}")"
TOT_MISS_SLOW_SEC="$(_f "${MISS_SLOW_SEC:-60.0}")"
# PN（來源 stot_trial.sh:21-38）
TOT_PN="${PN:-True}"
# ⚠️ PN 接戰距離。
#
# 【2026-08-13 從 4800 改為 5600】
# 舊註解寫「成敗看 gate_r/環半徑 比,4800/6084 = 0.789 是 TWO1 驗證過的那一點」。
# ⚠️⚠️ 那個「比值」在互動路徑上**沒有解釋力** —— RPK/RPL 十二架次全部是
#    gate/環 = 0.781、交接距離 4765–4787 m(差 ±11 m),比值零變異,
#    結果卻是 2/6。比值分不開成敗,所以它不是這條路徑上的自變數。
#
# 真正在變的是**交接時的高度**:實測 2286–3510 m(名目 perch_alt 3100,
# 有五架超調、最高 +410 m)。交接距離固定 ⇒ 需求俯角跟著散開 25.6°–36.3°:
#
#     γ ≤ 29.1°  三架殘餘 0.75 / 1.23 / 4.06 m   ✅
#     γ ≥ 33.8°  四架殘餘 412 / 623 / 860 / 1513 m ✗
#
# 對照 u1-descent-authority:P91 需求 29–33° 時 PN 追到 0.1–0.6° 內、六發全中;
# SW1 需求 64–69° 時只做到 53–66°、全脫靶 ⇒ **PN 在 ~33° 以上追不動**。
#
# 因為 γ需求 = atan(h / gate_r) 而 h 有上界、gate_r 沒有,拉大 gate_r 的作用
# **不是多給時間,是壓低最壞情況的陡度**:
#
#     4800 → 最壞 36.1°(現行,已在失敗帶)
#     5600 → 最壞 32.0°   ← 採用
#     5780 → 貼上環半徑 6084 的 95%,PN 會在放行瞬間接手而退化
#
# 可行窗口 5394–5780 由 contract_table.py 的兩條不變式夾出來
# (`gate_r 需求俯角 ≤ PN 可追蹤上限`、`gate_r < 環半徑`),改這個值或
# perch_alt / dive_angle 任一個,契約層都會重新驗算並在起飛前擋下。
# ⚠️ 拉大 gate_r **單獨進不了**實測全中的 ≤29.1° 帶(超調 13% 會把 γ 頂回
#    30–32°)。要進全中帶必須同時收斂交接高度的散布 —— 那是下一步。
#
# 【2026-08-13 當日回退 5600 → 4800:RPO 三趟實測反證】
# ⚠️⚠️ 上面那段推論**只看了 γ,漏了 gate 與守環擺盪的關係**,實測代價三趟:
#
#     RPK/RPL(gate 4800 / hold_radius 400):交棒距 4765–4787 m(±11)、真落地 5/12
#     RPO    (gate 5600 / hold_radius 1300):交棒距 3430–5579 m(±1075)、真落地 1/6
#
# 機制:守環是 loiter,飛機距紅心的水平距在 [環半徑 ± hold_radius] 之間擺盪。
#   hold_radius 400 ⇒ 擺盪 5747–6547,gate 4800 穩穩在外 ⇒ 每架都在進場途中乾淨穿越
#   hold_radius 1300 ⇒ 擺盪 4847–7447,gate 5600 **落在帶子裡** ⇒ 放行時常已在
#   gate 內側 ⇒ 當場交棒 ⇒ **交棒距離變成隨機量**,而交棒幾何是命中的唯一自變數。
#
# ⚠️ 兩個參數**單獨看都合法**(5600 在 [2500,6000] 內且 < 0.95×環=5780;
#    1300 ≥ 物理迴轉半徑 1253),相乘才出事。contract_table.py 已補上不變式
#    `gate_r < 環半徑 − hold_radius`(此組態為 6147−1300 = 4847),
#    並附迴歸鎖 TestHandoverGeometry::test_neither_parameter_alone_looks_wrong。
# ⚠️ hold_radius=1300 **要保留**:它讓交接高度散布從 1224 m 腰斬到 631 m,那一項有效。
TOT_GATE_R="$(_f "${GATE_R:-4700.0}")"

# ══ 2026-08-14 大整改:觀測層 / 安全兜底 / 下降式等待 ═════════════════
# ⚠️⚠️ **全部預設 = 現行行為**。這一組是為了讓每一項都能單獨開關、單獨歸因;
#    本專案 2026-08-13 剛因為『三項一起改』而無法判斷是哪一項造成的退步。
# ⚠️⚠️ **整數參數不可以走 _f()**。_f() 會補小數點(0 → 0.0),而 rclpy 的
#    型別檢查是嚴格的:宣告 INTEGER 卻餵 DOUBLE ⇒ InvalidParameterTypeException
#    ⇒ 節點**整個起不來**,而症狀是「tot_node 沒有進入 ready_for_target」——
#    看起來像節點壞了,不像型別問題。RPR 整批三趟因此報廢。
#    受影響的:param_verify / stall_prevention_glide / gate_hold_n(宣告都是 int)。
# ⚠️ 有參數沒有 -p 通路 = 純 no-op 而且零告警 —— pn_los_floor 就這樣
#    白費了 24 架次。下面每一個都在 tot_node 的 -p 清單裡,tests/ 有鎖。
TOT_PN_ALPHA_TAU="$(_f "${PN_ALPHA_TAU:-1.75}")"   # α 濾波時間常數(1.75 = 逐位元同舊行為)
TOT_TECS_PITCH_MIN="$(_f "${TECS_PITCH_MIN:-0.0}")"   # 0 = 跟隨 dive_pitch_min(舊行為);負值才拆開
TOT_STATE_MAX_AGE="$(_f "${STATE_MAX_AGE:-0.0}")"   # 0 = 停用狀態新鮮度看門狗
TOT_ATT_REQUIRE_FC="${ATT_REQUIRE_FC:-False}"   # False = 允許降級到分流線(舊行為)
TOT_PARAM_VERIFY="${PARAM_VERIFY:-0}"   # 0=關 1=讀回並印 2=失敗不准進 DIVE
TOT_ATT_HANDBACK="${ATT_HANDBACK:-False}"   # False = 不做交棒重錨(舊行為)
TOT_ATT_EAS_FLOOR="$(_f "${ATT_EAS_FLOOR:-0.0}")"   # 0 = 停用 EAS 地板 ⚠️ 用 EAS 不是地速
TOT_ATT_AGL_FLOOR="$(_f "${ATT_AGL_FLOOR:-0.0}")"   # 0 = 停用 AGL 地板
TOT_STALL_PREV_GLIDE="${STALL_PREV_GLIDE:-0}"   # 0 = 不動 .parm 的 STALL_PREVENTION
TOT_ATT_REQUIRE_GUIDED="${ATT_REQUIRE_GUIDED:-False}"   # False = 不檢查飛控模式(舊行為)
TOT_HOLD_ALT="$(_f "${HOLD_ALT:-0.0}")"   # 0 = 用 perch_of()(舊行為);>0 = 等待段降到這個高度
TOT_HOLD_V="$(_f "${HOLD_V:-0.0}")"   # 0 = 用 vmin(舊行為);>0 = 守環速度,讓油門離開地板
# ── ⭐ T1:用預測器的積分 t_go 取代靜態幾何配速(2026-08-14)──────────
# ⚠️ False = 沿用 `d / _pace_of`(現行)。散布 = 0.76×交接時間差 + 1.62,
#    而靜態式的尺度因子 SF=1.24、偏置中位 +11.6 s ⇒ 數學上達不到 ≤1.8 s。
TOT_TGO_FROM_PREDICTOR="${TGO_FROM_PREDICTOR:-False}"
TOT_TGO_PREDICT_DT="$(_f "${TGO_PREDICT_DT:-0.5}")"   # 積分節流(秒)
# ── ⭐ 把「還要多久才進得了 pushover 環」併入 t_go(RPZ1 2026-08-15)────
# ⚠️⚠️ 放行是**先空間閘門、後時間時刻表**:必須先飛進環才有資格放行。
#    而 dive_T = max_j(t_go_j)+2 裡的 t_go 只算直飛紅心的時間 ⇒ 對還在環外的
#    機體嚴重低估。RPZ1 的 UAV2 在環外 851 m,時刻表早就輪到它卻等了 37 s,
#    UAV1 已先走 ⇒ 放行時刻差 +37.2 s ⇒ 散布 36.2 s;
#    而同一趟的**俯衝耗時差只有 −1.0 s** —— 下游修好了,問題整個搬到上游。
# 文獻依據:Bin et al. 式 49,可行命中時刻下界由「當下可飛的最短幾何航程」
#    決定,不是直線距離;有空間閘門時那段航程 = 到環 + 環內俯衝。
TOT_TGO_RING_TRANSIT="${TGO_RING_TRANSIT:-True}"
# ── ⭐⭐ ITCG:命中時間控制(2026-08-15)────────────────────────────────
# 使用者定調:傳統 PN 是一打一,式子裡沒有時間 ⇒ 多機同時到達本質上做不到。
# 實測反證:RQA2 兩架放行幾何幾乎相同(5120m/31° 與 5590m/29°、放行差 −2.5s),
#   仍飛出 γ 中位 −27.1° vs −32.4°、耗時 **55s vs 33s** ⇒ PN 自己散開 21 s。
# 誤差用**公尺**形式 e_t = V·(t_d − t) − R(Kim & Shin 2023, Aerospace 10(3):308
#   式 5,逐字):三個量全部直接可測,**不需要估 t_go** ——
#   而 t_go 是本專案吃過兩次虧的量(時基污染 17.8 億秒、靜態式低估 2 倍)。
# ⚠️ ITCG_ON 預設 **False**:先把 RQA/RQB 的結果確認完再開,不要同時動兩個變數。
#    增益則**一律餵值**(即使開關關著)—— 讓契約表擋得住節點的泡棉預設 0.0,
#    同時保證真要開的時候不會吃到 0(= 開了但沒效果,最難查的一種)。
TOT_ITCG_ON="${ITCG_ON:-False}"
TOT_ITCG_GAIN="$(_f "${ITCG_GAIN:-1.0}")"
TOT_ITCG_A_MAX_G="$(_f "${ITCG_A_MAX_G:-0.4}")"
# ── ⚗️ 因果實驗:只在指定前置角放行(空字串 = 停用,生產組態)────────────
# ⚠️ 這條路徑**不是生產邏輯**。存在只為了回答一個問題:
#    俯衝耗時 σ=6.46 s 的變異,是放行**航向**造成的還是放行**距離**造成的?
#    現有 27 架次裡乾淨的那 4 架同時滿足「航向 27°」與「距離 6760 m」,完全共線。
# ⭐ 幾何免費對照:從紅心看盤旋圓的兩條切線,切點到紅心的距離都是
#    sqrt(D² − a²) —— **完全相同**,但一個切點朝向紅心、一個背對。
#    實算 D=6147 / a=1300 ⇒ 兩點都在 6058 m,σ 分別 2° 與 178°
#    ⇒ 距離自動被控制住,只有航向不同,共線問題天然消失。
#    (LM 建議的 2×2 全因子在圓形盤旋上取不到:R 與 σ 本來就耦合。)
# 用法:RELEASE_SIGMA_SPEC="1:0:20,2:180:20"
# ⚠️⚠️ 預設**不可以是空字串**。`-p name:=` 空值會讓 rcl 解析失敗:
#    「Couldn't parse parameter override rule」⇒ 節點整個起不來,
#    而症狀是「tot_node 沒有進入 ready_for_target」—— 長得跟埠號被搶走一樣。
#    RRA(2026-08-16)整批 8 趟、約 5 小時就是這樣報廢的:
#    RQE~RQH 都有傳這個變數所以沒事,**只有走預設值的生產路徑會炸**。
#    ⇒ 用哨兵字串,解析端把它當停用。
# ⚠️⚠️ 哨兵**不可以**用 "off"/"on"/"true"/"yes"/"-" —— rcl 會把它們解析成
#    **布林**,而參數宣告是 STRING ⇒ InvalidParameterTypeException ⇒ 一樣起不來。
#    實測(2026-08-16):"off" 型別錯誤 2 次、"-" 1 次;"none"/"disabled" 才乾淨。
TOT_RELEASE_SIGMA_SPEC="${RELEASE_SIGMA_SPEC:-none}"
# ⚠️⚠️ 餓死保護。RQE(2026-08-15)第一版沒有這個:對準組等 σ≤8° 等了 84 s,
#    而 miss 窗(dive_T+60 s)先關了 ⇒ **對照組一筆資料都沒有**,
#    而 log 只寫「逾時 MISS」,看起來像導引失敗 —— 實際上是實驗閘門把飛機餓死。
#    逾時就放行,但會 ERROR 標記「這一架次要剔除」。0 = 不逾時(不建議)。
TOT_RELEASE_SIGMA_TIMEOUT="$(_f "${RELEASE_SIGMA_TIMEOUT:-0.0}")"
# ── ⭐⭐ V1:用速度取代環上盤旋等待(2026-08-16)──────────────────────
# 實測:同一段 3.24 km,直線穿過 21–25 s(迂迴 0.99×),盤旋等待 114–200 s(3.0–4.7×)。
# 兩架時刻差在這 3 km 裡從 **4.4 s 放大到 40.2 s** —— 30→10 km 完全沒惡化,離散全在這。
# ⇒ 盤旋是用成本 100+ 秒的機制去吸收 4.4 秒,而過程本身製造 40 秒新誤差。
# 改成直線飛向環點 + 速度調節抵達時刻(3.24 km / 95–157 m/s ⇒ 13.5 s 吸收力,需求 4.4)。
# ⚠️ n=8:速度調節每架獨立、不搶共用資源;盤旋則是七架擠同一個環,放不下。
TOT_RING_PACE="${RING_PACE:-False}"
# ⭐⭐ S1(2026-08-18):抵環即放行 + 內切即翻相。兩顆都預設 **False**
#    ⇒ 關閉時逐位元同舊行為,一次 git checkout 就回得去。
# ⚠️ 預設值不可以是空字串 —— `-p name:=` 會讓 rcl 解析失敗、節點整個起不來,
#    而症狀長得像埠號被搶走(RRA 整批 8 趟 / 5 小時就是這樣報廢的)。
TOT_RELEASE_AT_RING="${RELEASE_AT_RING:-False}"
TOT_DIVE_FROM_RUNUP="${DIVE_FROM_RUNUP:-False}"
# ⭐⭐ S2:wait 段也飛過去。⚠️ 預設 False = 逐位元同 RRD。
TOT_WAIT_GOTO_RADIUS="${WAIT_GOTO_RADIUS:-False}"
# ── ⭐⭐ B1(2026-08-19):Jiang 2021 MPNG 的 Eq.(41) 時間項 ────────────
# ⚠️ 預設 **False** = 逐位元同現行的 surplus/σ_need 路徑,一顆開關就回得去。
# ⚠️ MPNG 掛在 `_itcg_bias` 裡 ⇒ 要它出力就**必須同時** ITCG_ON=True,
#    否則第一行就 return、六欄儀表全空,而契約仍會印出「MPNG 已啟用」。
#    契約層有不變式 _inv_mpng_needs_itcg_on 擋這個組合。
# ⚠️ 預設值不可以是空字串 —— `-p name:=` 會讓 rcl 解析失敗、節點整個起不來,
#    而症狀長得像埠號被搶走(RRA 整批 8 趟 / 5 小時就是這樣報廢的)。
TOT_MPNG_ON="${MPNG_ON:-False}"
# K_ε / K_s(1/(m·s))。0 = 走論文建議式 40/(r₀·tgo₀),由放行當拍的 R、t_go 自動推。
# ⚠️ 論文那個數字是他們 330 m/s **等速**、r₀=10 km 的算例;代進我們的實測工作點
#    (r₀≈6854 m、tgo₀≈41.3 s、V≈161.7 m/s、N=4)得 K≈1.41e−4 ⇒ a_s = 14.78·ε_T,
#    ε_T 只要 0.27 s 就撞上 0.4 g 的夾制 ⇒ 這條律在我們身上**整段貼在夾制上**、
#    退化成 bang-bang。⇒ 要掃的是 ITCG_A_MAX_G(契約區間 0.1–1.0),不是這兩個。
TOT_MPNG_K_EPS="$(_f "${MPNG_K_EPS:-0.0}")"
TOT_MPNG_K_S="$(_f "${MPNG_K_S:-0.0}")"
# ── ⭐ S9 影子預測:每架每 1/PREDICT_HZ 秒跑一次終端預測,**只記錄不致動** ──
# ⚠️⚠️ 這條從 S9 做好之後就沒接線:節點端 declare 了 predict_hz,但這裡從來
#    沒餵過 ⇒ 一路吃預設 0.0 ⇒ `_predict_tick()` 第一行就 return,
#    pred_* 五個欄位整批是空的,而且 early-return **不會有任何告警**。
#    校準好的預測器到 RPY3(2026-08-14)為止一趟都沒真的跑過。
#    2.0 Hz 足夠:一次積分約 500 步、幾個 ms,而預測量的變化尺度是秒。
TOT_PREDICT_HZ="$(_f "${PREDICT_HZ:-2.0}")"
# ── ⭐ T3:時間誤差閉迴路(蛇形吸收「太早到」)────────────────────────
# ⚠️⚠️ 開這個**必須同時把 WEAVE_PERIOD 拉大**。換向率 ω = g·tanφ/V:
#    157 m/s 只有 3.58 °/s,把航向從 +θ 甩到 −θ 要 2θ/ω。
#      P=22 s  ⇒ θ ≤ 9.8°  ⇒ 只吸收 t_go 的 1.5%(t_go 30 s ⇒ 0.45 s)
#      P=120 s ⇒ θ ≤ 53.7° ⇒ 吸收 41.4%(≈12 s)
#    22 是泡棉機時代的值。開了 T3 卻不改週期 = 看起來有做、實際吸不到時間。
TOT_WEAVE_TIME_SYNC="${WEAVE_TIME_SYNC:-False}"
TOT_WEAVE_THETA_MAX="$(_f "${WEAVE_THETA_MAX:-45.0}")"
TOT_WEAVE_DEADBAND_S="$(_f "${WEAVE_DEADBAND_S:-0.5}")"

# ── ⭐ 動態交棒的 gate_* (2026-08-14 補通路)──────────────────────────
# ⚠️⚠️ 這五個先前**整份 swarm_ros2_up.sh 都沒有 -p** ⇒ 在互動/產品路徑上
#    `export GATE_THETA_MAX=35` 是**純 no-op**,節點照吃宣告值 0.0,
#    而 log 仍印「(絕對 gate_r)」—— 開與沒開長得一模一樣。
#    全 repo 只有 stot_trial.sh 有通路,所以只有批次路徑能用。
# ⚠️ 預設維持 0.0 = 關閉:打開是行為改動,要單獨一趟驗。
TOT_GATE_THETA_MAX="$(_f "${GATE_THETA_MAX:-0.0}")"   # 0 = 關閉動態交棒(舊行為)
TOT_GATE_PHI="$(_f "${GATE_PHI:-0.80}")"   # 可行性餘裕:gate = d_min/Φ
TOT_GATE_R_MIN="$(_f "${GATE_R_MIN:-2500.0}")"   # 交棒距下限
TOT_GATE_R_MAX="$(_f "${GATE_R_MAX:-6000.0}")"   # 交棒距上限
TOT_GATE_HOLD_N="${GATE_HOLD_N:-3}"   # 連續幾拍成立才交棒(防單拍雜訊)
TOT_N_LAT="$(_f "${N_LAT:-4.0}")"
TOT_N_VERT="$(_f "${N_VERT:-5.0}")"
TOT_PN_LEAD="$(_f "${PN_LEAD:-1.0}")"
# ⚠️ 終端 PN 的速度上限。135 是泡棉機時代。來源 run_west.sh:234。
TOT_PN_VMAX="$(_f "${PN_VMAX:-210.0}")"
TOT_FREEZE_R="$(_f "${FREEZE_R:-100.0}")"
TOT_FREEZE_TGO="$(_f "${FREEZE_TGO:-1.5}")"
TOT_GATE_PROFILE="${GATE_PROFILE:-False}"
TOT_GATE_H="$(_f "${GATE_H:-350.0}")"
TOT_STAGGER_DEPART="${STAGGER_DEPART:-False}"
TOT_DIVE_FULL_THR="${DIVE_FULL_THR:-False}"
TOT_PN_LATE_FULL="${PN_LATE_FULL:-True}"
# ⚠️⚠️ 目標地表 AMSL。**海上目標一定要 0.0** —— SRTM 在開闊水域沒有資料，
#    回的外插值讓六架各報一個高度（實測散布 17 m，而命中門檻只有 10 m）。
#    這裡的預設維持 .nan（查地形庫，陸地目標用）；右鍵送目標時可在訊息裡帶
#    {"amsl": 0.0} 逐發覆寫，不必重啟節點。
#    ⚠️ 預設值必須寫 YAML 的 .nan；寫成 nan 會被解析成字串，節點型別檢查直接掛。
TOT_TARGET_AMSL="${TARGET_AMSL:-.nan}"
# ⭐ 扇形方位自動反推（由紅心看蜂群質心）。右鍵任意點時 approach_from=180
#    幾乎一定是錯的 —— 見 tot_node 的 auto_approach 說明。
TOT_AUTO_APPROACH="${AUTO_APPROACH:-True}"
TOT_APPROACH_FROM="$(_f "${APPROACH_FROM:-180.0}")"   # auto_approach 拿不到定位時的後備

echo "[ros2] origin=$BLAT,$BLON hdg=$BHDG | N=$N | LAUNCH=$LT | bio=$BIO"
echo "[ros2] manual=$MANUAL patrol=$AP_PATROL raft=$RAFT_ARG | wait=$WAIT_SPEED formed_r=$FORMED_RADIUS catch=$CATCH_SPEED"
echo "[ros2] tot_node=$TOT_ENABLE（STANDBY）| link $TOT_LINK_BASE+${TOT_LINK_STRIDE}i |" \
     "r_fan=$TOT_R_FAN perch=$TOT_PERCH_ALT dive_angle=$TOT_DIVE_ANGLE pn=$TOT_PN"

# ── 前置檢查：SITL 層分流埠要在 ────────────────────────────────────
if ! ss -ltn 2>/dev/null | grep -q ":5860 "; then
  echo "[ros2] ERROR: 分流埠 5860 沒在 LISTEN —— 先跑 bash swarm_sitl_up.sh" >&2
  exit 1
fi

# ── ⭐⭐ 機體身分守門（讀回式，不改任何預設）────────────────────────
# ⚠️⚠️ 本層的 tot_node 參數（VMIN 95 / v_plan 135 EAS / DIVE_SPEED 210 /
#    PN_VMAX 210 / perch 3100 / 俯角 27°）**全部假設 SITL 是改裝過的噴射機**。
#    但 `swarm_sitl_up.sh` **從不自己設 JET=1**（:64 是 `${JET:-0}`）——
#    只有 stot_repeat.sh / stot_kpi_loop.sh 會設。照本檔開頭 CMD1 的寫法
#    裸跑，拿到的是**原廠泡棉 plane**（AIRSPEED_MAX 30/35）：
#      ⇒ 上面每一發 DO_CHANGE_SPEED 都會被 ArduPlane 判 MAV_RESULT=4 **整發拒收**，
#        而且完全靜默（catch-speed-rejected-by-fc 那條教訓的第三次重演）。
#    再一種更陰的：`JET=1` 但 **JET_CDP 空** ⇒ 噴射模型 + **原廠阻力**，
#    平飛天花板只有 EAS 98.5（run_west.sh:66-67 的 G1 實測），
#    指令 135 永遠達不到、油門飽和、SEP≈0 —— 不會有任何錯誤訊息，只會慢。
#
# ⇒ 這裡**不改 JET 的預設**（翻成 1 會砸掉 swarm_sitl_up.sh 刻意保留的泡棉模式
#   與本檔 PRESET=tight/max 兩個組態），改成**讀回機體身分再比對**：
#   資料源是 arduplane 自己的 cmdline 與 SITL 層寫下的狀態檔，
#   **完全不建立 MAVLink 連線** —— 連一個 socket 都不開，不會去搶任何分流埠。
# ⚠️ 只在 TOT_ENABLE=1 時檢查：PRESET=tight/max 本來就是要泡棉機的。
# ⚠️ 逃生門 AIRFRAME_FORCE=1，而且訊息裡會印出來 —— 沒有逃生門的檢查，
#    下一個人會直接把整段註解掉。
if [ "$TOT_ENABLE" = "1" ] && [ "${AIRFRAME_FORCE:-0}" != "1" ]; then
  _af_model="$(pgrep -a arduplane 2>/dev/null \
                | sed -n '1s/.*--model \([^ ]*\).*/\1/p' || true)"
  _af_bad=""
  case "$_af_model" in
    "")          _af_bad="讀不到 arduplane 的 --model（SITL 還沒起完？）" ;;
    *-jet*)      : ;;                      # plane-jet / plane-jet-catapult
    *)           _af_bad="SITL 跑的是 **$_af_model**（原廠泡棉機）" ;;
  esac
  # 阻力改裝：MODEL 後面要掛 ":jet_model.json"，那是 swarm_sitl_up.sh:96 寫的
  if [ -z "$_af_bad" ]; then
    case "$_af_model" in
      *:*json)   : ;;
      *)         _af_bad="SITL 是噴射模型但**沒有低阻力改裝**（JET_CDP 空）" ;;
    esac
  fi
  if [ -n "$_af_bad" ]; then
    echo "[ros2] ✗ 機體身分不符：$_af_bad" >&2
    echo "[ros2]   本層要下的速度指令（v_plan 135 EAS / DIVE_SPEED $TOT_DIVE_SPEED /" >&2
    echo "[ros2]   PN_VMAX $TOT_PN_VMAX）是照**改裝噴射機**算的。" >&2
    echo "[ros2]   泡棉機的 AIRSPEED_MAX 只有 30/35 ⇒ 每一發 DO_CHANGE_SPEED 都會被" >&2
    echo "[ros2]   飛控判 MAV_RESULT=4 **整發拒收**，而且不會有任何錯誤訊息。" >&2
    echo "[ros2]   原廠噴射機（無 JET_CDP）則是平飛天花板只有 EAS 98.5，指令 135 永遠達不到。" >&2
    echo "[ros2]   處置：用改裝參數重跑 SITL 層 ——" >&2
    echo "[ros2]     JET=1 JET_CDP=0.01642 JET_THRMAX=25 LOC=TAOYUAN \\" >&2
    echo "[ros2]       N=$N SWARM_COUNT=$N bash swarm_sitl_up.sh" >&2
    echo "[ros2]   真的要用目前這個機體（例如 PRESET=tight 的泡棉編隊測試）：" >&2
    echo "[ros2]     AIRFRAME_FORCE=1 bash swarm_ros2_up.sh   （或 TOT_ENABLE=0）" >&2
    exit 1
  fi
  echo "[ros2] ✔ 機體身分：$_af_model（噴射 + 低阻力改裝）"
fi
# ⚠️ tot_node 走 6960+10i，那是 swarm_sitl_up.sh 另外開的一組 tcpin。
#    沒開的話 mavutil.wait_heartbeat 會空等 8 s × 6 台然後**帶著空的 link 起來**
#    —— 節點活著、狀態永遠湊不滿、右鍵指定目標毫無反應。先擋。
if [ "$TOT_ENABLE" = "1" ] && ! ss -ltn 2>/dev/null | grep -q ":$TOT_LINK_BASE "; then
  echo "[ros2] ERROR: tot_node 的分流埠 $TOT_LINK_BASE 沒在 LISTEN。" >&2
  echo "[ros2]   swarm_sitl_up.sh 應開四路：5860+（MAVROS）/ 6760+（AeroPlan）" >&2
  echo "[ros2]   / 6860+（tot_node）/ 6960+（編隊層 WP）。" >&2
  echo "[ros2]   ⚠️ 最可能的原因：SITL 層是 2026-08-07 之前起的，那時還沒有 6860 這一路。" >&2
  echo "[ros2]   處置：重跑 swarm_sitl_up.sh（會一併重開分流）。" >&2
  echo "[ros2]   ⚠️ 不要退回 TOT_LINK_BASE=6960 —— 那個埠編隊層先佔著，" >&2
  echo "[ros2]   tot_node 會變第二 client 收到 0 bytes，而且完全沒有錯誤訊息。" >&2
  echo "[ros2]   （臨時繞道請用 TOT_ENABLE=0 先不起 tot_node。）" >&2
  exit 1
fi

# ── 殘留的 tot_node：分辨「待命服務」與「批次航次」，不要無差別殺 ─────
# ⚠️⚠️ 兩個 tot_node 同時活著 = 兩套 DO_REPOSITION 互相蓋，而且完全靜默。
#    但無差別 pkill 一樣不行：stot_trial.sh:414 記著實測教訓 —— 無差別
#    `pkill -x tot_node` 把正在俯衝的 R451 殺在距共同命中時刻 2.5 秒，
#    整趟被記成「timeout / 觸地 0/6」，資料在方法學上全部作廢。
# ⇒ 用 cmdline 分辨：待命服務帶 `tlat:=.nan`，批次航次帶數字。
#    待命的 → 收掉（是上一輪自己留下的）；批次的 → **拒絕啟動**，把處置說清楚。
# ⚠️ pgrep/pkill 的 pattern 用 `[t]ot_node` 的括號寫法：命令列裡出現裸的
#    進程名會自我匹配（本專案的既有慣例，見下面那個 for 迴圈）。
# ⚠️⚠️ 2026-08-07：括號寫法**還不夠**。`pgrep -f 'tot_node'` 比對的是整條命令列，
#    所以任何「命令列裡提到 tot_node」的行程都會中 —— 實測被 `tail -F … |
#    grep -E '…|✗ tot_node|…'` 這種 log 監看指令打中，而它當然沒有 `tlat:=.nan`
#    ⇒ 被歸類成「批次航次的節點」⇒ **整個 ROS2 層拒絕啟動**，訊息還叫你去
#    tail trial_*.log（那裡什麼都沒有）。
#    ⇒ 收緊成 `tot_node --ros-args`：這是**真的節點**才會有的形狀
#      （`ros2 run sar_swarm tot_node --ros-args …` 與
#       `…/lib/sar_swarm/tot_node --ros-args …` 兩種都含）。
#      迴圈裡再驗一次 `--ros-args`，讓分類永遠只看真節點。
_tot_idle=""; _tot_batch=""
for _pid in $(pgrep -f '[t]ot_node --ros-args' 2>/dev/null || true); do
  _cl="$(tr '\0' ' ' < "/proc/$_pid/cmdline" 2>/dev/null || true)"
  case "$_cl" in
    *"--ros-args"*) : ;;                      # 真節點才往下分類
    *)              continue ;;
  esac
  case "$_cl" in
    *"tlat:=.nan"*) _tot_idle="$_tot_idle $_pid" ;;
    *)              _tot_batch="$_tot_batch $_pid" ;;
  esac
done
if [ -n "$_tot_batch" ]; then
  echo "[ros2] ✗ 偵測到**帶目標**的 tot_node 還在跑（PID:$_tot_batch）。" >&2
  echo "[ros2]   那是批次航次（stot_trial.sh / stot_kpi_loop.sh）的節點，可能正在俯衝。" >&2
  echo "[ros2]   兩個 tot_node 同時下 DO_REPOSITION 會互相蓋，而且沒有任何錯誤訊息。" >&2
  echo "[ros2]   處置：等那一趟結案（tail -f ~/fixwing_ws/runs/trial_*.log），" >&2
  echo "[ros2]         或確定要中止就自己 kill（本腳本不替你決定）。" >&2
  echo "[ros2]   真的要無視：TOT_FORCE=1 bash swarm_ros2_up.sh" >&2
  if [ "${TOT_FORCE:-0}" != "1" ]; then exit 1; fi
  echo "[ros2]   TOT_FORCE=1 —— 強制收掉 $_tot_batch"
  kill $_tot_batch 2>/dev/null || true
fi
if [ -n "$_tot_idle" ]; then
  echo "[ros2] 收掉上一輪待命的 tot_node（PID:$_tot_idle）"
  kill $_tot_idle 2>/dev/null || true
fi

# ── 清理殘留（只清本層：mavros + 節點；不動 SITL/mavproxy）──────────
pkill -x mavros_node 2>/dev/null || true
for p in "[m]aster_node" "[w]ingman_node" "[b]ackup_node" "[a]pm_sitl"; do
  pkill -f "$p" 2>/dev/null || true
done
sleep 2
# ⚠️⚠️ trap 裡的 tot_node **用行程群組殺，不用 pkill**：
#    (1) `ros2 run` 是 CLI wrapper，節點是它的**子行程**（ros2run/api/__init__.py
#        的 Popen + communicate）⇒ 只 kill wrapper 的話節點還活著，會跟下一輪
#        搶控制，而且沒有任何錯誤訊息（stot_trial.sh:421-431 記著這個坑）。
#    (2) 無差別 pkill 會誤殺平行跑的批次航次（同上）。
#    setsid 已經把 tot_node 關進自己的 session/PGID，負號 PID 只涵蓋本輪的子孫。
# ⚠️ $TOT_PGID 在 trap 觸發時才展開（單引號），所以寫在賦值之前是對的。
trap 'echo; echo "[ros2] 停止 ROS2 層（SITL 不動）..."; pkill -x mavros_node 2>/dev/null; for p in "[m]aster_node" "[w]ingman_node" "[b]ackup_node" "[a]pm_sitl"; do pkill -f "$p" 2>/dev/null; done; if [ -n "${TOT_PGID:-}" ]; then echo "[ros2] 收掉 tot_node 行程群組 -$TOT_PGID"; kill -TERM -"$TOT_PGID" 2>/dev/null; fi; exit 0' INT TERM

# ── 1) 6×MAVROS（連分流 5860+10i）─────────────────────────────────
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash"
echo "[ros2] 1) 啟動 $N 台 MAVROS（FCU_TCP_BASE=5860）"
FCU_TCP_BASE=5860 COUNT="$N" bash "$BR/start_mavros_swarm.sh" > /tmp/mavros_all.log 2>&1 &
# ⚠️⚠️ 2026-08-07:原本只等 **UAV1** 就 break,而訊息卻宣稱「啟動 N 台」。
#    ANM1 事故的鄰居:UAV2 的 MAVROS 若沒起來,僚機節點拿不到自己的定位,
#    而整條鏈路不會有任何人發現(對照 swarm_sitl_up.sh 是逐台驗證的)。
# ⚠️ 逾時從 40 放大到 90:內層每台最多 2 s,N 台單輪就可能 2N 秒,
#    六機一輪 12 s ⇒ 40 s 只夠三輪。這是 stot_trial.sh 記過的教訓:
#    「修好一個靜默失效的保護之後,要重新檢查它的參數」。
echo "[ros2]    等待 $N 台 MAVROS 的 /uavI/mavros/state connected..."
_mav_ok=0
for _t in $(seq 1 90); do
  _mav_ok=0
  for _i in $(seq 1 "$N"); do
    st=$(timeout 2 ros2 topic echo --once "/uav${_i}/mavros/state" 2>/dev/null \
         | grep -c "connected: true" || true)
    [ "${st:-0}" -ge 1 ] && _mav_ok=$((_mav_ok + 1))
  done
  [ "$_mav_ok" -ge "$N" ] && { echo "[ros2]    MAVROS $_mav_ok/$N connected"; break; }
  sleep 1
done
if [ "$_mav_ok" -lt "$N" ]; then
  echo "[ros2] ⚠️⚠️ MAVROS 只有 $_mav_ok/$N 連上 —— 缺的那幾架**不會被編隊層控制**,"
  echo "[ros2]      而且飛行中不會有任何訊息。繼續往下跑,但本趟的編隊結果不可採信。"
fi

# ── 2) 長僚機節點（角色依 N 分配）──────────────────────────────────
#    N>=2：master=UAV1 / wingman=UAV2..N-1 / backup=UAV N
#    N==2：master=UAV1 / **無僚機** / backup=UAV2（一長一備長)
#    ⚠️ 2026-08-07 訂正:舊註解寫「N==2 無 backup(兩架時編隊比失效接管重要)」
#       —— 那個前提是錯的。backup_node 會飛長機正後方的 V 尾 slot、發
#       formation_status,**它是編隊成員**;n=2 的一長一備長就是二機編隊,
#       而且多了失效接管。見 sar_failover_nodes.launch.py 的 _roles()。
#    ⚠️⚠️ SWARM_COUNT 一定要 export：launch 檔的角色分配要在「建 description 的
#       當下」跑 for 迴圈，只能走環境變數（LaunchConfiguration 要執行期才解析）。
#       少了這一行，N=2 也會生出 UAV3/4/5 的僚機與 UAV6 的備用長機 —— 那四架
#       不存在，節點會安靜地等一個永遠不會出現的機體，而 `ros2 node list`
#       看起來一切正常。
export SWARM_COUNT="$N"
echo "[ros2] 2) sar_swarm 長僚機節點（N=$N）"
ros2 launch sar_swarm sar_failover_nodes.launch.py \
    use_mavros:=true use_biomimetic:="$BIO" launch_type:="$LT" \
    auto_arm:="$AA" auto_takeoff:="$AT" auto_patrol:="$AP_PATROL" \
    with_raft:="$RAFT_ARG" \
    origin_lat:="$BLAT" origin_lon:="$BLON" heading_deg:="$BHDG" altitude:="$ORIGIN_ALT" \
    nominal_speed:="$NOMINAL_SPEED" wait_speed:="$WAIT_SPEED" \
    close_time:="${CLOSE_TIME:-60.0}" \
    climb_gate_hi:="${CLIMB_GATE_HI:-1000.0}" climb_cap_lo:="${CLIMB_CAP_LO:-1.5}" \
    cruise_alt_threshold:="${CRUISE_ALT_THRESHOLD:-3000.0}" \
    cruise_gate_hyst_m:="${CRUISE_GATE_HYST:-150.0}" \
    cruise_gate_dwell_s:="${CRUISE_GATE_DWELL:-5.0}" \
    airspeed_min:="${AIRSPEED_MIN_FC:-25.0}" \
    airspeed_max:="${AIRSPEED_MAX_CAP:-230.0}" \
    l1_lookahead_m:="${L1_LOOKAHEAD:-1500.0}" \
    cruise_roll_limit_deg:="${CRUISE_ROLL_LIMIT:-0.0}" \
    roll_limit_restore_deg:="${ROLL_LIMIT_RESTORE:-${DIVE_ROLL_MAX:-80.0}}" \
    master_alt_gate:="${MASTER_ALT_GATE:-0.0}" \
    formed_radius:="$FORMED_RADIUS" catch_speed:="$CATCH_SPEED" \
    catch_radius:="$CATCH_RADIUS" \
    v_spacing_m:="$V_SPACING" v_angle_deg:="$V_ANGLE" v_tail_dist_m:="$V_TAIL" \
    formation_shape:="${SHAPE:-V}" cone_half_deg:="${CONE_HALF:-30.0}" \
    formation_alt:="${FORM_ALT:-3100.0}" layer_sep:="${LAYER_SEP:-15.0}" \
    patrol_alt:="${PATROL_ALT:-${FORM_ALT:-3100.0}}" \
    wp_interval:="$WP_INTERVAL" wp_move_thresh:="$WP_MOVE_THRESH" \
    patrol_box_m:="$PATROL_BOX" formed_hysteresis:="$FORMED_HYST" \
    wp_lead_time:="$WP_LEAD" turn_radius:="$TURN_RADIUS" preslow_margin:="$PRESLOW_MARGIN" \
    > /tmp/nodes.log 2>&1 &
sleep 3

# ── 3) tot_node（終端導引）：STANDBY 待命，等 /swarm/tot_target ────────
TOT_PID=""; TOT_PGID=""
if [ "$TOT_ENABLE" = "1" ]; then
  mkdir -p "$TOT_OUT_DIR"
  echo "[ros2] 3) tot_node（終端導引）—— STANDBY 待命，log：$TOT_LOG"
  # ⚠️ tlat/tlon 明確寫 .nan，不靠節點的預設值。這一行是**待命語意的宣告**：
  #    看 cmdline 就知道這是待命服務而不是批次航次（上面的殘留偵測靠它分辨）。
  setsid nohup ros2 run sar_swarm tot_node --ros-args \
    -p n:="$N" -p tlat:=.nan -p tlon:=.nan \
    -p talt:="$TOT_TALT" -p approach_from:="$TOT_APPROACH_FROM" \
    -p auto_approach:="$TOT_AUTO_APPROACH" \
    -p r_fan:="$TOT_R_FAN" -p half_angle:="$TOT_HALF_ANGLE" \
    -p vmin:="$TOT_VMIN" -p vmax:="$TOT_VMAX" \
    -p plan_frac:=0.9 -p margin:="$TOT_MARGIN" -p perch_alt:="$TOT_PERCH_ALT" \
    -p ground_alt:="$TOT_GROUND_ALT" \
    -p dive_alt:=0.0 -p dive_layer:="$TOT_DIVE_LAYER" -p dive_angle:="$TOT_DIVE_ANGLE" \
    -p auto_ring:=True -p perch_alt_gate:=0.9 -p align_tol:="$TOT_ALIGN_TOL" \
    -p track_frac:="$TOT_TRACK_FRAC" -p dive_radius:="$TOT_DIVE_RADIUS" \
    -p dive_pace:="$TOT_DIVE_PACE" -p dive_spdweight:="$TOT_DIVE_SPDW" \
    -p overshoot:="$TOT_OVERSHOOT" -p switch_r:="$TOT_SWITCH_R" \
    -p dive_speed:="$TOT_DIVE_SPEED" -p reach:="$TOT_REACH" -p layer_m:="$TOT_LAYER_M" \
    -p perch_reach:="$TOT_PERCH_REACH" -p impact_reach:="$TOT_IMPACT_REACH" \
    -p hold_radius:="$TOT_HOLD_RADIUS" -p goto_radius:="$TOT_GOTO_RADIUS" \
    -p pn_alpha_tau:="$TOT_PN_ALPHA_TAU" -p tecs_pitch_min:="$TOT_TECS_PITCH_MIN" \
    -p state_max_age:="$TOT_STATE_MAX_AGE" -p att_require_fc:="$TOT_ATT_REQUIRE_FC" \
    -p param_verify:="$TOT_PARAM_VERIFY" -p att_handback:="$TOT_ATT_HANDBACK" \
    -p att_eas_floor:="$TOT_ATT_EAS_FLOOR" -p att_agl_floor:="$TOT_ATT_AGL_FLOOR" \
    -p stall_prevention_glide:="$TOT_STALL_PREV_GLIDE" -p att_require_guided:="$TOT_ATT_REQUIRE_GUIDED" \
    -p hold_alt:="$TOT_HOLD_ALT" -p hold_v:="$TOT_HOLD_V" \
    -p tgo_from_predictor:="$TOT_TGO_FROM_PREDICTOR" -p tgo_predict_dt:="$TOT_TGO_PREDICT_DT" \
    -p predict_hz:="$TOT_PREDICT_HZ" -p tgo_ring_transit:="$TOT_TGO_RING_TRANSIT" \
    -p itcg_on:="$TOT_ITCG_ON" -p itcg_gain:="$TOT_ITCG_GAIN" \
    -p itcg_a_max_g:="$TOT_ITCG_A_MAX_G" \
    -p mpng_on:="$TOT_MPNG_ON" -p mpng_k_eps:="$TOT_MPNG_K_EPS" -p mpng_k_s:="$TOT_MPNG_K_S" \
    -p ring_pace:="$TOT_RING_PACE" -p release_at_ring:="$TOT_RELEASE_AT_RING" -p dive_from_runup:="$TOT_DIVE_FROM_RUNUP" -p wait_goto_radius:="$TOT_WAIT_GOTO_RADIUS" -p release_sigma_spec:="$TOT_RELEASE_SIGMA_SPEC" -p release_sigma_timeout:="$TOT_RELEASE_SIGMA_TIMEOUT" \
    -p weave_time_sync:="$TOT_WEAVE_TIME_SYNC" -p weave_theta_max:="$TOT_WEAVE_THETA_MAX" -p weave_deadband_s:="$TOT_WEAVE_DEADBAND_S" \
    -p gate_theta_max:="$TOT_GATE_THETA_MAX" -p gate_phi:="$TOT_GATE_PHI" -p gate_r_min:="$TOT_GATE_R_MIN" \
    -p gate_r_max:="$TOT_GATE_R_MAX" -p gate_hold_n:="$TOT_GATE_HOLD_N" \
    -p line_tol:="$TOT_LINE_TOL" -p resend_min_dt:="$TOT_RESEND_DT" \
    -p impact_alt_ok:="$TOT_IMPACT_ALT_OK" -p impact_settle:="$TOT_IMPACT_SETTLE" \
    -p miss_far_m:="$TOT_MISS_FAR_M" -p miss_far_k:="$TOT_MISS_FAR_K" \
    -p miss_window_s:="$TOT_MISS_WINDOW_S" \
    -p miss_slow_gs:="$TOT_MISS_SLOW_GS" -p miss_slow_sec:="$TOT_MISS_SLOW_SEC" \
    -p target_amsl:="$TOT_TARGET_AMSL" \
    -p perch_climb_v:="$TOT_PERCH_CLIMB_V" -p dive_pitch_min:="$TOT_DIVE_PITCH_MIN" \
    -p dive_pitch_max:="$TOT_DIVE_PITCH_MAX" -p dive_angle_max:="$TOT_DIVE_ANGLE_MAX" \
    -p pn_los_floor:="$TOT_PN_LOS_FLOOR" -p pn_nz_min:="$TOT_PN_NZ_MIN" \
    -p weave_period:="$TOT_WEAVE_PERIOD" \
    -p stagger_depart:="$TOT_STAGGER_DEPART" -p stagger_lead:="$TOT_STAGGER_LEAD" \
    -p perch_step:="$TOT_PERCH_STEP" -p dive_full_thr:="$TOT_DIVE_FULL_THR" \
    -p dive_thr_max:="$TOT_DIVE_THR_MAX" \
    -p pn_thr_trim:="$TOT_PN_THR_TRIM" -p pn_thr_gain:="$TOT_PN_THR_GAIN" \
    -p pn_late_full:="$TOT_PN_LATE_FULL" -p dive_hdem_tconst:="$TOT_HDEM_TC" \
    -p perch_band:="$TOT_PERCH_BAND" -p perch_settle:="$TOT_PERCH_SETTLE" \
    -p pn_enable:="$TOT_PN" -p gate_r:="$TOT_GATE_R" \
    -p pn_n_lat:="$TOT_N_LAT" -p pn_n_vert:="$TOT_N_VERT" -p pn_lead:="$TOT_PN_LEAD" \
    -p pn_vmax:="$TOT_PN_VMAX" -p pn_freeze_r:="$TOT_FREEZE_R" \
    -p pn_freeze_tgo:="$TOT_FREEZE_TGO" \
    -p gate_profile:="$TOT_GATE_PROFILE" -p gate_h:="$TOT_GATE_H" \
    -p csv:="$TOT_CSV" \
    -p link_base:="$TOT_LINK_BASE" -p link_stride:="$TOT_LINK_STRIDE" \
    > "$TOT_LOG" 2>&1 < /dev/null &
  TOT_PID=$!
  # ⚠️ PGID **讀回來**，不要假設它等於 $!（本專案鐵則：不要相信自己送出的值）。
  #    讀不到就退回用 $!，至少不會比只 kill wrapper 差。
  TOT_PGID="$(ps -o pgid= -p "$TOT_PID" 2>/dev/null | tr -d ' \t')"
  case "$TOT_PGID" in ''|*[!0-9]*) TOT_PGID="$TOT_PID";; esac
  echo "[ros2]    ros2 run wrapper PID=$TOT_PID / 行程群組 PGID=$TOT_PGID"
  # ⚠️⚠️ **讀回來驗證它真的進了 STANDBY。** 「起了但沒生效」是本專案代價最高
  #    的一類錯：節點可能在 wait_heartbeat 卡 48 s、可能因參數型別崩掉，
  #    而背景啟動看起來都一樣成功。等到那一行待命宣告出現才算數。
  _tot_ok=0
  for _t in $(seq 1 60); do
    # ⚠️⚠️ 這個字串的合約來源是 sar_swarm_patches/tot_node.py 的 STANDBY_SENTINEL。
    #    刻意不含全形括號 —— 全形「（」與半形「(」在終端機裡幾乎看不出差別，
    #    而 grep 失配的後果是「明明待命成功卻報失敗」。
    #    tests/test_tot_standby.py 比對兩邊，改一邊沒改另一邊會直接紅。
    if grep -q "STANDBY 待命就緒" "$TOT_LOG" 2>/dev/null; then _tot_ok=1; break; fi
    if ! kill -0 "$TOT_PID" 2>/dev/null; then break; fi
    sleep 1
  done
  if [ "$_tot_ok" = "1" ]; then
    echo "[ros2]    ✔ tot_node 已進 STANDBY（等 /swarm/tot_target）"
    # ⚠️⚠️ 「進了 STANDBY」**不等於**「接得了目標」。待命宣告只證明節點活著；
    #    真正的門檻是 ready_for_target = (收到定位的機數 >= N)。
    #    2026-08-07 實測：分流埠被編隊層先佔走時，這行 ✔ 照樣印，
    #    但 n_pos 恆 0 ⇒ AeroPlan 右鍵打擊按下去毫無反應，log 也不再有任何一行。
    #    所以這裡**再讀一次真正的就緒旗標**，不要只信宣告。
    _rdy=0
    for _t in $(seq 1 40); do
      if timeout 3 ros2 topic echo --once /swarm/tot_status 2>/dev/null \
           | grep -q '"ready_for_target": true'; then _rdy=1; break; fi
      sleep 1
    done
    if [ "$_rdy" = "1" ]; then
      echo "[ros2]    ✔ tot_node ready_for_target=true（$N 台定位齊備，可接目標）"
    else
      echo "[ros2]    ⚠️ tot_node 在 STANDBY，但 **ready_for_target 仍是 false**。" >&2
      echo "[ros2]      這時候發 /swarm/tot_target（含 AeroPlan 右鍵打擊）不會有任何動作。" >&2
      echo "[ros2]      最可能原因：分流埠 $TOT_LINK_BASE+${TOT_LINK_STRIDE}i 被別的 client" >&2
      echo "[ros2]      先佔走 —— MAVProxy 的 --out=tcpin 只餵第一個連上的 client。" >&2
      echo "[ros2]      查證：ss -tnpi | grep -A1 tot_node   （沒有 bytes_received 就是 0）" >&2
      echo "[ros2]      ⚠️ 6960+10i 是編隊層的，tot_node 必須走 6860+10i。" >&2
    fi
  else
    echo "[ros2]    ✗ tot_node 60 s 內沒有進 STANDBY —— **不要以為它在待命**。" >&2
    echo "[ros2]      最後 20 行 $TOT_LOG：" >&2
    tail -20 "$TOT_LOG" 2>/dev/null >&2
    echo "[ros2]      常見原因：(a) $TOT_LINK_BASE+${TOT_LINK_STRIDE}i 沒有 $N 個 tcpin ⇒ wait_heartbeat 逐台空等；" >&2
    echo "[ros2]                (b) 某個 -p 少了小數點 ⇒ InvalidParameterTypeException；" >&2
    echo "[ros2]                (c) sar_swarm 沒重建 ⇒ 跑到舊版的 tot_node.py（deploy_build.sh）。" >&2
  fi
fi

echo "[ros2] ═══════════════════════════════════════════════════"
echo "[ros2] ROS2 層就緒（常駐）。"
if [ "$MANUAL" = "1" ]; then
  echo "[ros2] 待命模式：飛機在地面 disarmed，起飛由你觸發："
  echo "[ros2]     bash swarm_takeoff.sh          # 全部（間隔 STAGGER=5s）"
  echo "[ros2]     bash swarm_takeoff.sh 1        # 只放長機"
fi
echo "[ros2] 監看：ros2 topic echo /swarm/heartbeat   節點 log：tail -f /tmp/nodes.log"
if [ "$TOT_ENABLE" = "1" ]; then
  echo "[ros2] ── 終端導引 tot_node（STANDBY）────────────────────"
  echo "[ros2]   狀態：ros2 topic echo /swarm/tot_status     log：tail -f $TOT_LOG"
  echo "[ros2]   指定目標（AeroPlan 右鍵會送同一則）："
  echo "[ros2]     ros2 topic pub --once /swarm/tot_target std_msgs/String \\"
  echo "[ros2]       '{data: \"25.1406000,121.2325000\"}'"
  echo "[ros2]   海上目標請帶 amsl（SRTM 在水域沒資料，散布 17 m）："
  echo "[ros2]     '{data: \"{\\\"lat\\\":25.14,\\\"lon\\\":121.23,\\\"amsl\\\":0.0}\"}'"
  echo "[ros2]   ⚠️ 打完會自己回 STANDBY，可以接第二個目標；第 2 發的 CSV 是"
  echo "[ros2]      ${TOT_CSV}_s2.csv（不覆蓋第 1 發）。"
  echo "[ros2]   ⚠️⚠️ swarm_ros2_down.sh **收不掉 tot_node**（它只認 master/wingman/"
  echo "[ros2]      backup/mavros）。停本層請用 Ctrl+C（本腳本的 trap 會殺整個行程"
  echo "[ros2]      群組 -$TOT_PGID）；已經斷開的話：kill -TERM -$TOT_PGID"
fi
echo "[ros2] 停止本層（SITL 不動）：Ctrl+C（或 bash swarm_ros2_down.sh，見上方 ⚠️）"
echo "[ros2] ═══════════════════════════════════════════════════"
wait
