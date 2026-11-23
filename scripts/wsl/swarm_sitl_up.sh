#!/usr/bin/env bash
# swarm_sitl_up.sh — 第 1 層：6 台 SITL + MAVProxy 分流，常駐（不含任何 ROS2）。
#
# 與 swarm_ros2_up.sh 成對使用（SITL / ROS2 控制分離）：
#   終端 A：bash swarm_sitl_up.sh          # SITL 層常駐（本腳本）
#   終端 B：bash swarm_ros2_up.sh          # ROS2 層（6×MAVROS + 長僚機節點）
# ROS2 層可隨時 swarm_ros2_down.sh 停掉重開，SITL 完全不受影響（飛機還在飛）。
#
# 分流埠（每台 i=0..N-1）：
#   SITL master  tcp 5760+10i（MAVProxy 是唯一 client）
#   MAVROS       tcpin 5860+10i
#   AeroPlan     tcpin 6760+10i（嵌入式蜂群 base_port 6760）
#   tot_node     tcpin 6860+10i（STOT 導引；**必須獨佔**，見 _launch_bridge 的說明）
#   編隊層 WP    tcpin 6960+10i（master_node / backup_node，每台各連自己那一個）
#
# ⚠️ 一個 tcpin 埠只餵**第一個**連上的 client。所以「同時要跑的兩個消費者」
#    永遠不可以共用一個埠 —— 這是本專案最貴的一類靜默失敗。
#
# 用法：
#   bash swarm_sitl_up.sh                    # 桃園機場、6 台、跑道起飛
#   LOC=HUWEI bash swarm_sitl_up.sh          # 虎尾
#   LOC=23.5,120.3,0,90 N=4 SPACING=50 ...   # 自訂
#   LAUNCH=catapult bash swarm_sitl_up.sh    # 彈射（frame plane-catapult）
#
# 生成參數會寫入 ~/fixwing_ws/sitl_run/swarm_state.env，
# swarm_ros2_up.sh 讀它對齊 origin / launch_type（避免編隊原點 ≠ 生成點的順移 bug）。
#
# 停止：bash swarm_sitl_down.sh（或 Ctrl+C；會一併收掉 mavproxy）
# ⚠️ 結尾 wait 常駐 —— SITL 設 PR_SET_PDEATHSIG，本腳本就是它們的命；別關這個終端。
set -eo pipefail

AP="$HOME/ardupilot/build/sitl/bin/arduplane"
BASE="$HOME/fixwing_ws/sitl_run"
BR="$HOME/fixwing_ws/src/sar_swarm/scripts"
STATE="$BASE/swarm_state.env"
export PATH="$HOME/.local/bin:$PATH"

# ── 出生地點預設（與 AeroPlan config/spawn_presets.py 同址）────────────
resolve_loc() {
  case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
    TAOYUAN|RCTP|TPE|"") echo "25.0777,121.2325,32,52" ;;  # 桃園機場 跑道05（預設）
    HUWEI)      echo "23.702732,120.419333,0,0"  ;;        # 虎尾科大
    SWARM)      echo "22.60,120.00,0,90" ;;                # 舊蜂群原點
    *)          echo "$1" ;;                               # 已是 lat,lon,alt,hdg
  esac
}
LOC_RAW="${LOC:-TAOYUAN}"
LOC="$(resolve_loc "$LOC_RAW")"
IFS=',' read -r BLAT BLON BALT BHDG <<<"$LOC"

N="${N:-6}"
SPACING="${SPACING:-30}"          # 縱列間距 [m]

case "$(echo "${LAUNCH:-runway}" | tr '[:upper:]' '[:lower:]')" in
  catapult) MODEL="plane-catapult"; LT="catapult" ;;
  *)        MODEL="plane";          LT="runway"   ;;
esac

# ── JET=1：高速噴射物理模型（plane-jet，巡航 40 / 最大 ~80 m/s）──────
# 泡棉機 plane 上限 ~30 m/s；要更快必須換 plane-jet 物理模型。這裡並把
# AIRSPEED_MAX / ARSPD_FBW_MAX 解到很高（AIRSPEED_MAX_CAP，預設 120），讓它
# 飛到模型物理極限而非被參數卡住。實際上限由 plane-jet 氣動/推力決定（~80-100）。
JET_BASE="$HOME/ardupilot/Tools/autotest/default_params/plane-jet.parm"
if [ "${JET:-0}" = "1" ]; then
  # SIM_Plane 用獨立的 strstr 解析 frame 字串（-jet / -catapult 各自成立），
  # 所以噴射＋彈射要合成 "plane-jet-catapult"。
  # ⚠️ 舊版此處直接 MODEL="plane-jet"，會把 LAUNCH=catapult 選到的模型蓋掉 →
  #    彈射器 silently 消失（have_launcher=false），只能跑道滾行起飛。
  if [ "$LT" = "catapult" ]; then MODEL="plane-jet-catapult"; else MODEL="plane-jet"; fi
  # ── 高速改裝(JET_CDP / JET_THRMAX)──────────────────────────────────
  # 把 plane-jet 改成指定的「平飛頂速 / 俯角終端速度」。兩顆旋鈕就夠,而且
  # **都不需要重編譯 ArduPilot**:
  #
  #   阻力   → JSON 覆寫 c_drag_p。`Plane::load_coeffs()` 的觸發條件是 frame 字串
  #            含 ':' 且以 '.json' 結尾;而且它在建構子裡比 `-jet` 區塊**更早**執行,
  #            `-jet` 只動 mass/thrust_scale,所以係數不會被蓋回去。
  #   推重比 → `THR_MAX`。`calculate_forces()` 算的是 servo 輸出 × thrust_scale,
  #            而 servo 上限就是 THR_MAX ⇒ THR_MAX=19 等於可用推力只剩 19%。
  #
  # ⚠️ **不要試圖用 mass 改推重比**:thrust_scale = mass·g / hover_throttle 與質量
  #    成正比,所以 T/W ≡ 1/hover_throttle = 1.43 是恆定的,加重完全不影響。
  #    而 (V_終端/V_平飛)² = 1 + (W/T)·sinγ,T/W=1.43 時即使垂直俯衝倍率上限也只有
  #    1.30 —— 想要更大的倍率,唯一的路就是把 THR_MAX 壓下來。
  #
  # 數值請用 scripts/wsl/jet_design.py 反推,例:
  #   巡航 136 / 終端 204 @ 20°  →  JET_CDP=0.01150  JET_THRMAX=19  (perch 2900)
  #
  # ⚠️ **JSON 路徑必須是相對於各實例工作目錄的檔名,不能給絕對路徑。**
  #    `load_coeffs()` 用 `AP::FS().stat()` 找檔案,SITL 的檔案層不吃絕對路徑,
  #    給了就直接 `PANIC: <path> failed to load` 然後整個 arduplane 死掉
  #    (實測:絕對路徑 PANIC、同一份檔案放進 cwd 用裸檔名就
  #     「Loaded plane aero coefficients from ...」)。
  #    因為每台是 `cd uavN` 之後才 exec,所以下面的啟動迴圈會把這份 JSON
  #    複製進每個 uavN/。
  if [ -n "${JET_CDP:-}" ]; then
    printf '{ "c_drag_p": %s }\n' "$JET_CDP" > "$BASE/jet_model.json"
    MODEL="${MODEL}:jet_model.json"
    echo "[sitl] 高速改裝:c_drag_p=$JET_CDP → model=$MODEL"
  else
    # 沒指定就把上一輪留下的清掉,免得「以為在跑原廠機體,其實還帶著改裝係數」
    rm -f "$BASE/jet_model.json"
  fi
  # ⚠️⚠️ 150 會**卡死追趕通道**。ArduPlane 的 DO_CHANGE_SPEED 只在
  #    AIRSPEED_MIN ≤ 目標 ≤ AIRSPEED_MAX 時才接受,區間外 return false
  #    ⇒ MAV_RESULT_FAILED,整發丟掉。而 catch_speed = 157 > 150
  #    ⇒ 實測 V1:**每一發追趕指令都被拒收**,僚機只吃得到 nominal 135。
  #    這條是縱向唯一有權限的通道,等於一直是關的,而且完全靜默。
  #    這個參數的原意就是「解除空速上限、飛到模型物理極限」——
  #    它不該是個會綁住指令的值。
  # ⚠️⚠️ 但**不要一次拉太高**:實測 W1 拉到 220 之後階段 1 退步
  #    (UAV6 卡在 ~1200 m、climb 逾時,UAV1–5 正常)。AIRSPEED_MAX 會進
  #    TECS 的速度需求上限,動它就是動能量分配。
  #    ⇒ 只要**剛好蓋過 catch_speed(157)** 就夠了,取 165 留 8 m/s 餘裕。
  #      解除拒收是目的,改變 TECS 的工作點不是。
  #
  # ⚠️⚠️ 2026-08-07 更正:預設從 165 改成 **230**。上面那段 W1 的告誡仍然記在這裡,
  #    但它**已經被後來的實測蓋過**:
  #      * run_west.sh:86 匯出 `AIRSPEED_MAX_CAP=230`,經 stot_repeat.sh:73 傳到這裡
  #        ⇒ **每一趟批次航次(含唯一達標的 TWO1)實際跑的都是 230**,不是 165。
  #      * 而且兩者是耦合的:批次的 `DIVE_SPEED=210`(run_west.sh:273)在
  #        AIRSPEED_MAX=165 之下會被飛控**整發拒收**(MAV_RESULT=4)。
  #        也就是說「165」這個值和批次的俯衝組態根本不相容。
  #    改成 230 是為了讓**互動路徑(AeroPlan 右鍵打擊)與達標組態一致** ——
  #    在此之前,一個裸的 `JET=1 bash swarm_sitl_up.sh` 會給出一組沒有任何
  #    航次驗證過的 165/210 混搭。
  #    ⚠️ W1 的反證(拉到 220 之後 UAV6 climb 逾時)沒有被推翻,只是被更晚的
  #      證據取代。要回退:`AIRSPEED_MAX_CAP=165 bash swarm_sitl_up.sh`,
  #      但記得同時把 DIVE_SPEED 壓到 165 以下,否則俯衝指令會被拒。
  CAP="${AIRSPEED_MAX_CAP:-230}"
  # 姿態限制解除（噴射機不該被泡棉級的角度上限綁住）。DIVE=0 可關掉這段。
  # ⚠️ 用 .parm 在 SITL 啟動時載入，而非事後 param_set —— 分流讀回參數不可靠，
  #    「送出去了但沒吃進去」曾讓終端俯衝只掉 10 m/s（該有 ~71 m/s）而高空掠過。
  DIVE_PITCH_MIN="${DIVE_PITCH_MIN:--80}"   # 最大低頭角（負值；-80 = 幾乎垂直俯衝）
  DIVE_PITCH_MAX="${DIVE_PITCH_MAX:-45}"
  DIVE_ROLL_MAX="${DIVE_ROLL_MAX:-80}"
  # ⚠️⚠️ 這是「陡俯角掉不下去」的機體側成因,而且完全沒有訊息。
  #    TECS_SINK_MAX 是「最大下沉率**需求**」硬上限,反推可達航路角 = asin(SINK/V)。
  #    120 之下:V=219 → 只做得到 33.2°;V=258 → 27.7°。也就是說任何走 TECS 的
  #    下降段(gate_r 之外的航點段)**永遠做不到 45°**,不管俯角參數怎麼設。
  #    45° 的實際下沉率:收油門 219·sin45 = 155,滿推 258·sin45 = 182 m/s。
  #    注意 PN 段走 SET_ATTITUDE_TARGET 不吃 TECS,所以以前 gate_r 大、PN 幾乎
  #    全程接管時這條被遮住了 —— 現在 gate_r 收小、航點段回來了,它就會咬人。
  DIVE_SINK_MAX="${DIVE_SINK_MAX:-120}"
  # ⚠️ 預設 15 = ArduPlane 原廠值 = 現行實際行為。寫出來只為了讓它可見+進契約表。
  #    改法:`SCALING_SPEED=50 bash ...`,但先讀下面 jet_fast.parm 裡那段警告。
  SCALING_SPD="${SCALING_SPEED:-15}"
  # ⚠️ 定時炸彈:sitl_overrides.parm 只在 MAXV=1 且 JET≠1 時才寫,但**從不刪除**,
  #    而 JET=1 的 DEF_LIST 仍無條件收錄它。WSL 上現存一份泡棉機的殘留
  #    (AIRSPEED_MAX 35 / MIN 9 / CRUISE 27),每次 JET=1 開機都被載入。
  #    今天無害純粹是因為 jet_fast.parm 排最後、把三項都覆蓋回去;只要之後有人
  #    從 jet_fast 拿掉其中一項,35 m/s 的空速上限就會靜默生效。和清 jet_model.json
  #    同樣的理由,開跑前清掉。
  rm -f "$BASE/sitl_overrides.parm"
  cat > "$BASE/jet_fast.parm" <<EOF
# 由 swarm_sitl_up.sh JET=1 產生 — 解除空速上限，飛到模型物理極限
AIRSPEED_MAX $CAP
# ⚠️ ARSPD_FBW_MAX 在本 build **不存在**(已對照 Parameters.cpp 確認,只有
#    AIRSPEED_MAX)。留著會讓人以為「兩個空速上限都解開了」而誤判哪個沒吃進去。
ARSPD_USE 1
# ⭐⭐ 關掉空速計的「失效自動停用」。實測 P91/T1 兩趟都刷:
#      FCU: Airspeed sensor 1 failure. Disabling   (每機 1–24 次)
#    停用之後 TECS 改吃合成空速,DO_CHANGE_SPEED 的行為就不再可預期。
#    高空大側滑角讓皮托管動壓與 GPS 地速對不上,正是這個檢查的觸發條件。
#
# ⚠️⚠️ 參數名對照本 build 原始碼確認過:
#    `ARSPD_STRICT` 與 `SIM_ARSPD_FAIL` 在這個版本**都不存在**(會被靜默忽略)。
#    真正的開關是 AP_Airspeed_Health.cpp 讀的 ARSPD_OPTIONS:
#      bit0 ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE ← 就是它在 Disabling
#      bit3 USE_EKF_CONSISTENCY                 ← 一致性檢查的觸發源
#    設 0 = 兩個都關,感測器永遠不會被自動停用。
ARSPD_OPTIONS 0
# ⭐⭐ 舵面速度縮放的基準速度。`Attitude.cpp::calc_speed_scaler()`(逐字):
#      speed_scaler = g.scaling_speed / aspeed
#      speed_scaler = constrain_float(speed_scaler, scale_min, scale_max)
#    ⇒ 所有俯仰/滾轉 PID 的**有效增益 ∝ SCALING_SPEED / 空速**。
#
# ⚠️⚠️ 這一行**刻意寫死成與 ArduPlane 預設相同的 15**,行為逐位元不變。
#    寫出來的唯一目的是讓它**從隱形變成可見**並進契約表 ——
#    先前它從來沒被設過,也不在任何檢查裡,是「餵值路徑一斷就靜默退回」的
#    現成入口(與 pn_vmax 同類)。
#
# ⚠️ 改它之前必讀:它是**純乘子**,不改變 1/V 的形狀。
#    15 → 135 等價於**把所有俯仰/滾轉 PID 增益 ×9**,而且我們的速度區間
#    (95–210)在任何合理值下都碰不到 scale_min/scale_max 的夾限。
#    實測(RQF 前一批 dataflash):高速段升降舵只用 1400–1619 µs
#    (中立 1500、滿舵 1000/2000)⇒ **只用了約 ±12% 行程、0% 打到底**,
#    Pitch 追隨誤差中位 1.9–3.8° ⇒ **現有增益不缺權限**。
#    所以調高它主要是加振盪風險,不會修好「俯衝中權限衰減 43%」那件事
#    (那來自 1/V 本身)。要動請單獨一趟驗,不要直接進批次。
SCALING_SPEED $SCALING_SPD
# ⭐⭐ 實驗 A 的**唯一變數**:讓 TECS 在空速感測器被 EKF3 拒收時改用合成空速。
# ⚠️⚠️ 根因(R451 dataflash 分窗實測):3100 m 皮托管讀 115.2,而真值 EAS≈134.3,
#    比值 1.164 ≈ E2T 1.169 —— 讀值少了一個密度修正因子 ⇒ EKF3 判定不一致而拒收
#    ⇒ `AP_AHRS::using_airspeed_sensor()` = false
#    ⇒ `AP_TECS::use_airspeed()`(AP_TECS.cpp:1604-1610)= false
#    ⇒ ① `_SKE_weighting` 被強制 0(俯仰完全不看速度誤差)
#      ② 走 `_update_throttle_without_airspeed()`(油門公式裡沒有速度項)
#    ⇒ **DO_CHANGE_SPEED 全系統零權限**。
#    決定性證據:`TEC3`/`TECS` 這兩筆只在 `_update_throttle_with_airspeed()` 內寫入,
#    整段巡航+終端 **0/6400**;而節點被殺後飛機降到低空、E2T 1.169→0.999,
#    `CTUN.AsT` 立刻由 3(合成)翻回 1(感測器)、TEC3 由 0 變 0.996 ——
#    **同機同趟的天然對照,證明這是高度相依的失效,不是壞掉的感測器。**
#    行為層對應:v_cmd 指令跨度 45→104 m/s,實得地速幾乎不動
#    (PERCH/hold_ring 指令 45、實飛 160.4;DIVE/wait 指令 45、實飛 228.1)。
# ⇒ SYNAIRSPEED=1 讓 `use_airspeed()` 回 true(AP_TECS.h:448 `_use_synthetic_airspeed`),
#   `_update_throttle_with_airspeed()` 才會被呼叫,速度指令才拿得回權限。
# ⚠️ 這是**治標偏本**:真正的根因是皮托管讀值少一個 E2T 因子,治本要修
#    SITL 的空速模型或 ARSPD_RATIO。但 SYNAIRSPEED 是單一變數、可逆、
#    而且直接對準「速度通道有沒有權限」這個唯一要驗的問題。
TECS_SYNAIRSPEED ${TECS_SYNAIRSPEED:-1}
# 實作上 bit0 還要配 ARSPD_WIND_MAX>0 才會判 implausible;顯式歸零不留死角。
ARSPD_WIND_MAX 0
EOF
  if [ "${DIVE:-1}" = "1" ]; then
    cat >> "$BASE/jet_fast.parm" <<EOF
# ── 終端俯衝：解除角度/下沉限制（參數名已對照本 build 原始碼確認）──
PTCH_LIM_MIN_DEG $DIVE_PITCH_MIN
PTCH_LIM_MAX_DEG $DIVE_PITCH_MAX
ROLL_LIMIT_DEG $DIVE_ROLL_MAX
STALL_PREVENTION 0
TECS_SINK_MAX $DIVE_SINK_MAX
TECS_SINK_MIN 5
TECS_PITCH_MIN $DIVE_PITCH_MIN
TECS_PITCH_MAX $DIVE_PITCH_MAX
TECS_SPDWEIGHT 1
THR_MIN 0
EOF
  fi
  # ── 高速改裝的推重比與飛行包絡 ──────────────────────────────────────
  if [ -n "${JET_THRMAX:-}" ]; then
    cat >> "$BASE/jet_fast.parm" <<EOF
# 推重比 = THR_MAX(servo 上限就是可用推力比例);THR_MAX 是 AP_Int8,只吃整數。
THR_MAX $JET_THRMAX
# ⚠️ TRIM_THROTTLE 必須跟著 THR_MAX 下來。plane-jet.parm 的原廠值是 36,而我們把
#    THR_MAX 壓到 $JET_THRMAX —— TECS 的名目油門 nomThr = TRIM_THROTTLE/100 = 0.36
#    比自己的上限 0.14 還大,前饋值一送出就被削平,整個 TECS 主導段(俯衝前半、
#    pushover)油門通道等於開迴路飽和。俯角越陡、能量變化越劇烈,誤差越明顯。
#    ⚠️ 不可設成 ≤1:mode_guided 走 forced_throttle 的前提是 THR_MAX>0 且名目合理。
TRIM_THROTTLE $JET_THRMAX
# ⚠️ **起飛段不要一起壓。** 曾經把 TKOFF_THR_MAX 也設成 $JET_THRMAX,結果彈射
#    給的 30 m/s 加上只有 2.1 m/s² 的淨加速根本爬不出去 —— 飛機掉回地面滑行
#    (實測 throttle=19 已到頂、airspeed 8.6、rel_alt −23.8)。
#    彈射本來就是模擬用的外部助力,起飛推力放寬等同於「指定一具更大的彈射器」,
#    而且 TKOFF_THR_MAX 只在 TAKEOFF 模式生效:轉 GUIDED 後 THR_MAX 就接管,
#    巡航與俯衝的量測完全不受影響。
TKOFF_THR_MAX ${JET_TKOFF_THRMAX:-100}
EOF
  fi
  # ⚠️⚠️ **JET_CRUISE 必須有預設值。** 它為空時整個區塊被跳過,
  #    `AIRSPEED_CRUISE` 就留在原廠 plane-jet 的 **40 m/s** ——
  #    而那是 TECS 在「沒有生效的 DO_CHANGE_SPEED」時的目標空速。
  # ⚠️ 這個洞先前被另一個 bug 蓋住:`ARSPD_OPTIONS` 的自動停用位元會在飛行中
  #    把空速計關掉(P91 每機 2–12 次),TECS 改吃合成空速,於是**忽略**了 40。
  #    我把 `ARSPD_OPTIONS=0` 修好之後感測器一直健康,TECS 就真的去飛 40 了 ——
  #    實測 FULL1 俯衝段(<1500 m):EAS 41.2、油門中位數 **16%**,
  #    六機以 40–49 m/s 飄過紅心上空(水平只差 0.2–1.2 m,但高度差 19–1567 m)。
  #    ⇒ 歷次 197–205 m/s 的終端地速,有一部分是**靠空速計被自動停用換來的**。
  #    設計巡航是 135 EAS,JCRU = JET_CRUISE×0.85 ⇒ JET_CRUISE 取 159。
  JET_CRUISE="${JET_CRUISE:-159}"
  if [ -n "${JET_CRUISE:-}" ]; then
    # 迴轉半徑 ∝ V²:V²/(g·tan60°)
    JLOIT=$(awk "BEGIN{printf \"%d\", ($JET_CRUISE*$JET_CRUISE)/(9.80665*1.7320508)}")
    JCRU=$(awk "BEGIN{printf \"%d\", $JET_CRUISE*0.85}")
    cat >> "$BASE/jet_fast.parm" <<EOF
# ── 高速改裝的飛行包絡:不改這幾項會被舊機體的上限**靜默**卡住 ──
# ⚠️ AIRSPEED_MIN **不要**跟著巡航速度一起拉高。改裝只動阻力與推力,升力係數沒變,
#    失速速度仍然只有 ~23 m/s。曾經設成 40:彈射只給 30 m/s,飛控判定低於最小
#    空速就一路壓機頭補速度,離地後直接撞回地面。維持原廠的 25。
AIRSPEED_MIN 25
AIRSPEED_CRUISE $JCRU
# ⚠️⚠️ **低頭上限必須壓住,否則 TECS 會拿高度去換速度、直接把飛機開進地面。**
#    實測(jet_trace,轉 GUIDED @178 m):油門全程頂在 19%(已到 THR_MAX),
#    空速 67.8 → 105.6 一路**上升**(不是失速),但俯仰被壓到 −35°,
#    alt 227 → −5.9,11 秒撞地。
#    機制:TECS_SPDWEIGHT=1 時油門與俯仰共同分擔能量。這個機體 T/W 只有 0.274,
#    68 → 80 m/s 光靠油門要 6 秒,TECS 等不及就用俯仰去要位能;而下面的 DIVE
#    區塊為了終端俯衝把 TECS_PITCH_MIN 放到 −80°,於是它真的敢壓到 −35°。
#    原廠 T/W 1.43 油門不會飽和,所以舊組態從來沒暴露這一點。
#    ⇒ 巡航段把低頭限制收回 −20°;**終端俯衝所需的 −80° 由 tot_node 在進 DIVE
#      的當下才放開**(參數 dive_pitch_min),和它處理 TECS_SPDWEIGHT 的作法一致。
#    ⚠️ 這一段在 heredoc 裡,**不可以出現反引號** —— unquoted heredoc 會把它當成
#      命令替換執行掉,而且是靜默地把那段內容從輸出檔裡吃掉(實測:
#      「dive_pitch_min: command not found」,註解行整段消失)。這次只吃到註解,
#      但同樣的機制隨時可能吃掉一行真正的參數。
#    ⚠️ 這兩行和上面 DIVE 區塊的 PTCH_LIM_MIN_DEG 寫進**同一個檔**,後者勝 ——
#      所以 DIVE_PITCH_MIN 寫進 .parm 的那份是死的,開機值就是這裡的值。
#      要改終端低頭權限請改 tot_node 的 dive_pitch_min(runtime param_set),
#      改這裡只會改到巡航段。做成變數是為了讓「哪個值真正生效」看得見。
PTCH_LIM_MIN_DEG ${CRUISE_PITCH_MIN:--20}
TECS_PITCH_MIN ${CRUISE_PITCH_MIN:--20}
# ⚠️ TAKEOFF 模式預設只爬到 50 m 就平飛,交接到 GUIDED 時只有百來公尺高度可以賠
#    暫態。低推重比機體的能量暫態要好幾秒,拉高交接點才有餘裕。
TKOFF_ALT 300
# 抬頭方向同樣要配合推重比:可持續爬升角 = asin(T/W − D/W),T/W 0.274 之下
#   50 m/s → 12.8°、70 m/s → 11.1°、80 m/s → 9.9°(爬升率 13.8 最佳)、100 m/s → 6.9°
# 也就是最多只能爬 ~12°,45° 是要不到的。
#    ⚠️ 同上,終端要更大的抬頭權限請用 tot_node 的 dive_pitch_max,不是改這裡。
PTCH_LIM_MAX_DEG ${CRUISE_PITCH_MAX:-15}
TECS_PITCH_MAX ${CRUISE_PITCH_MAX:-15}
TECS_CLMB_MAX 13
# ⚠️ 舊值 180 m。$JET_CRUISE m/s / 60° 傾角的迴轉半徑是 $JLOIT m —— 180 m 物理上
#    轉不出來,PERCH 守環會變成永遠切不進去(飛機被捕獲進一個做不到的圓)。
WP_LOITER_RAD $JLOIT
WP_RADIUS 400
# L1 週期 17 s 在 $JET_CRUISE m/s 下的前置距離過大,轉彎會大幅超越航段
NAVL1_PERIOD 12
EOF
  fi
fi

# ── MAXV=1（泡棉專用）：解除 30 m/s 參數上限，配 ROS2 層 PRESET=max ─────────
# 泡棉 plane 的 30 是「參數」上限（models/plane.parm AIRSPEED_MAX 30）;氣動天花板
# 其實 ~31.5（SIM_Plane: 推力 28N = 阻力 @31.5-32）。解到 35 讓 TECS 用滿氣動極限。
# ⚠️ 此 build 參數已改名（post-4.4）：AIRSPEED_MAX/MIN、AIRSPEED_CRUISE（m/s）；
# 舊名 ARSPD_FBW_*/TRIM_ARSPD_CM 會被靜默忽略。
if [ "${MAXV:-0}" = "1" ] && [ "${JET:-0}" != "1" ]; then
  cat > "$BASE/sitl_overrides.parm" <<EOF
# 由 swarm_sitl_up.sh MAXV=1 產生 — 泡棉頂速（配 swarm_ros2_up.sh PRESET=max）
AIRSPEED_MAX 35
AIRSPEED_MIN 9
AIRSPEED_CRUISE 27
EOF
  echo "[sitl] MAXV=1：泡棉空速上限 30→35（氣動極限 ~31.5），cruise 27"
fi

# --defaults 疊層。JET=1 用 plane-jet.parm（噴射氣動/調參）取代 plane.parm，
# 再疊 jet_fast（解上限）；否則用 plane.parm（加速度校正）。
DEF_LIST=""
if [ "${JET:-0}" = "1" ]; then
  # ⚠️ models/plane.parm 必須先疊:它帶 12 行 INS_ACC* 加速度校正,而
  #    plane-jet.parm 一行都沒有。少了它 → 「PreArm: 3D Accel calibration needed」
  #    → 六機全部解不了鎖(彈射/起飛靜默失敗)。之後才疊 plane-jet 的噴射氣動調參。
  # ⚠️ jet_fast.parm **不在這裡疊**,它要排在最後面 —— 見下方 sitl_catapult.parm
  #    後的補疊。
  for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
           "$JET_BASE" "$BR/sitl_extra.parm" "$BASE/sitl_overrides.parm"; do
    [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
  done
else
  for f in "$HOME/ardupilot/Tools/autotest/models/plane.parm" \
           "$BR/sitl_extra.parm" \
           "$BASE/sitl_overrides.parm"; do
    [ -f "$f" ] && DEF_LIST="${DEF_LIST:+$DEF_LIST,}$f"
  done
fi
[ "$LT" = "catapult" ] && [ -f "$BR/sitl_catapult.parm" ] && \
  DEF_LIST="${DEF_LIST:+$DEF_LIST,}$BR/sitl_catapult.parm"
# ⚠️ jet_fast.parm 必須疊在**最後**。它是「解除上限 / 高速改裝」的覆寫層,語意上
#    就該壓過前面所有人。舊版把它排在 sitl_catapult.parm 之前,於是
#    sitl_catapult.parm 的 `TKOFF_ALT 100` 靜默蓋掉了改裝要的 300 ——
#    症狀是飛控照舊印「Takeoff to 100m」,而 jet_fast.parm 裡明明寫著 300。
#    這種覆寫沒有任何警告,只能靠讀回參數或看飛控訊息才會發現。
[ "${JET:-0}" = "1" ] && [ -f "$BASE/jet_fast.parm" ] && \
  DEF_LIST="${DEF_LIST:+$DEF_LIST,}$BASE/jet_fast.parm"

echo "[sitl] LOC=$LOC_RAW → $LOC | N=$N | SPACING=${SPACING}m | LAUNCH=$LT"
[ -x "$AP" ] || { echo "[sitl] ERROR: 找不到 $AP" >&2; exit 1; }

# ── 清理殘留（只清本層：arduplane + mavproxy）──────────────────────
pkill -x arduplane 2>/dev/null || true
pkill -f "[m]avproxy.py" 2>/dev/null || true
pkill -f "[t]ail -f /dev/null" 2>/dev/null || true
sleep 2
trap 'echo; echo "[sitl] 停止 SITL 層..."; pkill -x arduplane 2>/dev/null; pkill -f "[m]avproxy.py" 2>/dev/null; pkill -f "[t]ail -f /dev/null" 2>/dev/null; exit 0' INT TERM

# ── 寫狀態檔（swarm_ros2_up.sh 讀取以對齊 origin / launch_type / N）──
cat > "$STATE" <<EOF
# 由 swarm_sitl_up.sh 自動產生 — swarm_ros2_up.sh 讀此對齊參數
SWARM_LOC="$LOC"
SWARM_LAUNCH="$LT"
SWARM_N="$N"
EOF
echo "[sitl] 狀態檔已寫入: $STATE"

# ── 6×SITL（單縱列，沿跑道航向排開）───────────────────────────────
echo "[sitl] 啟動 $N 台 arduplane（model=$MODEL，單縱列）"
for i in $(seq 0 $((N - 1))); do
  n=$((i + 1))
  wd="$BASE/uav$n"; mkdir -p "$wd"; rm -f "$wd/eeprom.bin" "$wd/mav.parm"
  # 氣動 JSON 要放進實例目錄:MODEL 帶的是裸檔名(絕對路徑會讓 SITL PANIC)
  [ -f "$BASE/jet_model.json" ] && cp "$BASE/jet_model.json" "$wd/jet_model.json"
  home=$(awk -v lat="$BLAT" -v lon="$BLON" -v alt="$BALT" -v hdg="$BHDG" -v i="$i" -v s="$SPACING" \
    'BEGIN{ pi=3.141592653589793; br=(hdg+180.0)*pi/180.0; d=i*s;
            dlat=(d*cos(br))/111320.0; dlon=(d*sin(br))/(111320.0*cos(lat*pi/180.0));
            printf "%.7f,%.7f,%s,%s", lat+dlat, lon+dlon, alt, hdg }')
  printf '  UAV%-2d -I%-2d tcp:%-5d @ %s\n' "$n" "$i" "$((5760 + 10 * i))" "$home"
  ( cd "$wd" && exec "$AP" -w --model "$MODEL" --home "$home" \
       -I"$i" --sysid "$n" ${DEF_LIST:+--defaults "$DEF_LIST"} ) \
       > "/tmp/sitl_uav$n.log" 2>&1 &
done
up=0
for _t in $(seq 1 60); do
  up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((5760 + 10 * i)) " && up=$((up + 1)); done
  [ "$up" -ge "$N" ] && break; sleep 1
done
echo "[sitl] SITL ports up=$up/$N"

# ── MAVProxy 分流（stdin keepalive + 逐台驗證/重啟自癒）──────────────
# 半數 fan-out 起不來的成因：前一次 bring-up 的 mavproxy 沒清乾淨仍佔埠 →
# 新實例 bind 撞 [Errno 98] Address already in use 而死 → 只剩前幾台（AeroPlan
# 就會「有些連得到、有些連不到」）。對策：先等舊埠釋放，再啟動；啟動後以
# AeroPlan 埠 6760+10i 為準逐台驗證，沒起來的 kill 掉重啟，重試數輪。
echo "[sitl] MAVProxy 分流（MAVROS 5860+ / AeroPlan 6760+ / 編隊層 6960+ / tot_node 6860+）"
# ⚠️⚠️ 2026-08-07 修:tot_node 必須有**自己的**分流埠,不可以和編隊層共用 6960+10i。
#   `--out=tcpin` 只餵**第一個**連上的 client(實測見下),而編隊層的
#   master_node(UAV1→6960)、backup_node(UAV2→6970)都比 tot_node 早連上。
#   ⇒ tot_node 是第二個 client ⇒ **一個位元組都收不到**。
#
#   實測(2026-08-07,AeroPlan 互動航次,`ss -tnpi` 讀 socket 位元組計數):
#     master_node → 6960   bytes_received:847965      ← 第一 client,有餵
#     tot_node    → 6960   (無 bytes_received 欄位=0) ← 第二 client,餓死
#     tot_node    → 6970   (無 bytes_received 欄位=0)
#   後果:`ready_for_target` 恆 false ⇒ AeroPlan 右鍵打擊按下去沒反應,
#   而且 **完全靜默** —— tot_node 的「等機體定位中」告警只在 INIT 印,
#   STANDBY 走的是 `_standby_tick()` 那條 early return。
#
#   ⭐ 批次路徑(stot_trial.sh)為什麼沒撞到:它在交棒前先 `swarm_ros2_down.sh`
#      把整個編隊層收掉(stot_trial.sh:425),6960/6970 空出來,航次用的
#      tot_node 才變成第一 client。互動路徑是**兩層同時在跑**,沒有這個機會。
#      —— 這正是「批次會成功、產品路徑(右鍵打擊)在失敗帶」的機制。
# ⭐ 第四路:跨出 WSL 到 Windows 端的 X-Plane 12 / 地面站。
#   GCS_HOST=<Windows IP> bash swarm_sitl_up.sh
#   埠 = GCS_PORT + i(每機一條,預設 14550/14551/…)
#   ⚠️ 用 udpout(WSL 主動送出)而不是 tcpin —— WSL2 是 NAT,Windows 連不進來,
#      但 WSL 出得去。Windows IP 取 `ip route show default | awk '{print $3}'`。
#   ⚠️ 這一路是**唯讀觀測**用途。要從地面站下指令會和 tot_node / 編隊層搶控制,
#      那正是本專案踩過的「兩個控制器對同一架飛機發 DO_CHANGE_SPEED」。
[ -n "${GCS_HOST:-}" ] && echo "[sitl] + 外部輸出 udpout:$GCS_HOST:${GCS_PORT:-14550}+i（X-Plane / 地面站）"
mkdir -p /tmp/mavbridge

_launch_bridge() {   # $1 = instance index i
  local i="$1" n=$(( $1 + 1 ))
  local m=$((5760 + 10 * i)) mp=$((5860 + 10 * i)) ap=$((6760 + 10 * i)) np=$((6960 + 10 * i))
  local tp=$((6860 + 10 * i))
  setsid bash -c "tail -f /dev/null | mavproxy.py \
      --master=tcp:127.0.0.1:$m \
      --out=tcpin:0.0.0.0:$mp --out=tcpin:0.0.0.0:$ap --out=tcpin:0.0.0.0:$np \
      --out=tcpin:0.0.0.0:$tp \
      ${GCS_HOST:+--out=udpout:$GCS_HOST:$((${GCS_PORT:-14550} + i))} \
      --source-system=255 --non-interactive --state-basedir=/tmp/mavbridge \
      --logfile=/tmp/mavbridge/uav$n.tlog" >/tmp/mavbridge/uav$n.log 2>&1 &
}
_kill_bridge() {     # $1 = instance index i；殺該實例的 mavproxy（依 master 埠精準比對）
  local m=$((5760 + 10 * $1))
  for pid in $(pgrep -f "master=tcp:127.0.0.1:$m" 2>/dev/null); do kill -9 "$pid" 2>/dev/null; done
}

# 先確保目標 fan-out 埠都空著（等舊 mavproxy 釋放，最多 ~8s）
for _t in $(seq 1 8); do
  busy=0
  for i in $(seq 0 $((N - 1))); do
    ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " && busy=$((busy + 1))
  done
  [ "$busy" -eq 0 ] && break
  _kill_bridge_all() { for i in $(seq 0 $((N-1))); do _kill_bridge "$i"; done; }
  _kill_bridge_all; sleep 1
done

# 首發全部起
for i in $(seq 0 $((N - 1))); do _launch_bridge "$i"; done
# 驗證 + 重啟自癒（以 AeroPlan 埠為準；AeroPlan 實際連的就是這個）
for attempt in 1 2 3 4; do
  sleep 3
  miss=""
  for i in $(seq 0 $((N - 1))); do
    ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " || miss="$miss $i"
  done
  [ -z "$miss" ] && break
  echo "[sitl]   retry #$attempt：UAV$(for i in $miss; do printf '%s ' $((i+1)); done)分流未起，重啟"
  for i in $miss; do _kill_bridge "$i"; sleep 1; _launch_bridge "$i"; done
done
up=0; for i in $(seq 0 $((N - 1))); do ss -ltn 2>/dev/null | grep -q ":$((6760 + 10 * i)) " && up=$((up + 1)); done
echo "[sitl] AeroPlan 分流 ports up=$up/$N（6760..$((6760 + (N - 1) * 10))）"
[ "$up" -lt "$N" ] && echo "[sitl] ⚠ 仍有 $((N - up)) 台未起 —— 檢查 /tmp/mavbridge/uav<N>.log"

WSL_IP="$(hostname -I | awk '{print $1}')"
echo "[sitl] ═══════════════════════════════════════════════════"
echo "[sitl] SITL 層就緒（常駐）。WSL_IP=$WSL_IP"
echo "[sitl] AeroPlan：嵌入式蜂群 → host=$WSL_IP base_port=6760 stride=10 count=$N"
echo "[sitl] 下一步：另開終端跑 bash swarm_ros2_up.sh（讀 $STATE 自動對齊 origin）"
echo "[sitl] 停止本層：bash swarm_sitl_down.sh"
echo "[sitl] ═══════════════════════════════════════════════════"
wait
