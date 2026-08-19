#!/usr/bin/env bash
# stot_trial.sh — 跑「一趟」STOT 紅心命中,**假設 SITL 已經在跑**。
#
# 與 stot_kpi_loop.sh 的差別:那支每趟都重啟 SITL(95s)+ 完整逾時等待,適合無人
# 值守連跑;這支省掉重啟,適合「改一個參數 → 立刻看結果」的迭代。
# ⚠️ 一趟打完機體就消耗在地上了(expend_on_impact),要再跑一趟必須重啟 SITL。
#
# 用法: TRIAL=q1 bash stot_trial.sh
# ⚠️ 預設值 = 2026-07-27 驗證過的組態(U 批 5/5 三項全過),不帶任何環境變數
#    直接跑就會重現它。要對照舊的航點導引基準才需要 PN=False。
# 環境變數: PN / DIVE_PACE / GATE_R / N_LAT / N_VERT / PN_LEAD / DIVE_LAYER
#           TRACK_FRAC / ALIGN_TOL / DIVE_ANGLE / TRIAL(標籤) / CSV
# ⚠️ 不要 set -u:source ROS2 setup.bash 會踩到未設定的 AMENT_TRACE_SETUP_FILES。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
TRIAL="${TRIAL:-t1}"
CSV="${CSV:-/tmp/stot_kpi.csv}"
MISS_MAX="${MISS_MAX:-5}"
ALT_MAX="${ALT_MAX:-30}"
SPREAD_MAX="${SPREAD_MAX:-3.0}"
PN="${PN:-True}"
# ⚠️⚠️ 50.0 → 0(自動)。50 是泡棉機時代的值(vmin=45 時 50≈vmin,符合
#    「配保守一點」的設計意圖);噴射組態 vmin=95 之後 50 ≪ 95,時刻表比最低
#    可飛速度還慢 ⇒ 俯衝整段被 `max(vmin, …)` 夾住 ⇒ 同步權限歸零、
#    dive_speed 不生效。0 = min(vmax, 0.8·dive_speed)。見
#    tests/test_dive_pace_invariant.py 與 swarm_ros2_up.sh 同名變數的說明。
# ⚠️⚠️ 2026-08-14:預設從 `0` 改成 `0.0`。
#    `dive_pace` 在 tot_node 宣告成 0.0(DOUBLE),而這裡若傳整數 `0`,
#    rclpy 會丟 InvalidParameterTypeException ⇒ **節點整個起不來**,
#    症狀是「tot_node 沒有進入 ready_for_target」—— 看起來像節點壞了。
#    ⚠️ 這一行在 `_f()`(第 132 行,會自動補小數點)**定義之前**,
#       所以不能靠它,必須把小數點寫進字面值。
#    同族的反向錯誤剛讓 RPR 整批三趟報廢(整數參數被 _f() 補成 DOUBLE)。
#    兩個方向都由 tests/test_ros_param_types.py 自動檢查。
DIVE_PACE="${DIVE_PACE:-0.0}"
GATE_R="${GATE_R:-1200.0}"
N_LAT="${N_LAT:-4.0}"
N_VERT="${N_VERT:-5.0}"
PN_LEAD="${PN_LEAD:-1.0}"
PN_VMAX="${PN_VMAX:-135.0}"
# 交接盒剖面(預設關;見 tot_node 的 gate_profile 說明)
GATE_PROFILE="${GATE_PROFILE:-False}"
GATE_H="${GATE_H:-350.0}"
# ⚠️ 終端凍結:R 或 t_go 小於門檻就鎖住指令。**不要調小。**
# 它擋的是 PN 的奇異點:視線角速率 λ̇ = (de·vn − dn·ve)/d²,分母是 d²,
# R→0 時會爆掉、滾轉指令飽和、飛機亂飛。
# 實測 T1(收成 30 m / 0.6 s):觸地誤差 2083 m、散布 32 s,整批崩掉。
# 「最後 ~105 m 是開迴路」確實是殘餘 AGL 的來源,但要從**收斂速度**下手
# (pn_n_vert),不能從拆掉這個保護下手。
FREEZE_R="${FREEZE_R:-100.0}"
FREEZE_TGO="${FREEZE_TGO:-1.5}"
DIVE_LAYER="${DIVE_LAYER:-4.0}"
TRACK_FRAC="${TRACK_FRAC:-0.8}"
DIVE_SPDW="${DIVE_SPDW:-0.0}"
ALIGN_TOL="${ALIGN_TOL:-40.0}"
DIVE_ANGLE="${DIVE_ANGLE:-10.0}"
# ⭐ 終端俯衝地速主要來自位能:在環上 h 公尺、初速 v0,理想終端 v=√(v0²+2gh)。
#    perch 400 m(離紅心地表 443 m)理想值只有 ~106 m/s,扣掉阻力就到不了 100。
#    要穩定拿到 100 m/s 得把 perch 拉高。auto_ring 會據此重算 pushover 環半徑:
#    pushover_r = (perch_alt − dive_alt)/tan(dive_angle),要確認它仍落在 r_fan 內。
PERCH_ALT="${PERCH_ALT:-400.0}"
TALT="${TALT:-400.0}"
PERCH_BAND="${PERCH_BAND:-40.0}"
PERCH_SETTLE="${PERCH_SETTLE:-25.0}"
# 觸水終結:SITL 不會因為落海而終結機體,不強制 disarm 就會浮起來再飛一趟
# (TWO1 實測 UAV2 觸地後 11.7 s 爬回 +1103 m)。KILL_ON_TOUCHDOWN=False 可回退。
KILL_ON_TOUCHDOWN="${KILL_ON_TOUCHDOWN:-True}"
KILL_AGL_M="${KILL_AGL_M:-0.0}"
KILL_CONFIRM_N="${KILL_CONFIRM_N:-2}"
# 守環/飛越半徑。
# hold_radius=400 是泡棉機時代(60 m/s TAS)算的,噴射組態要 1252 m,
# 節點每次開機都用 ERROR 吼這件事卻沒人能改它 —— 因為以前沒有 -p 通路。
#
# 【2026-08-13 400 → 1300】通路是 2026-08-05 補的,但預設「維持節點原值
# (不改行為)」⇒ **通路補了、值沒改**,ERROR 又繼續吼了八天。
# 這正是 effective-value-contract 那一類:警告存在、沒有人擋、照跑。
#
# ⚠️ 這不只是清潔工作。守環退化的後果是「放行時距目標的距離會在
#    環半徑 ±迴轉半徑 之間隨機」(SW1:進環 d 一致 3108–3131,放行卻散到
#    1806–3449),而 RPK/RPL 剛好量到**交接高度散布 ±600 m**
#    ([[handover-altitude-is-the-variable]])—— 兩者極可能同源。
# 1300 = 迴轉半徑 1253(vmin 95 EAS @ 3100 m、45° 傾角)再留一點餘裕。
HOLD_RADIUS="${HOLD_RADIUS:-1300.0}"
GOTO_RADIUS="${GOTO_RADIUS:-30.0}"
# ⚠️ 動態交棒的 GATE_* 參數**不放這裡** —— 它們要用 _f()(自動補小數點),
#    而 _f() 定義在下面。放在定義之前會得到 `_f: command not found` + 空值,
#    而 `bash -n` 檢查不出來。見 _f() 那一段之後。
# ⚠️⚠️ 2026-08-14(S7)25.0 → 3000.0。
#    本檔號稱是「參數真相源」,但 LINE_TOL 這一項**從來沒有真的生效過**:
#    run_west.sh:226 一律 export 3000.0 蓋過它,所以批次路徑跑的是 3000。
#    而互動路徑(swarm_ros2_up.sh)照著本檔抄了 25.0 ⇒ 兩條路徑實際行為不同,
#    這正是 memory/jet-config-not-mirrored 記的那一類。
#
#    25.0 的後果(run_west.sh:216-218 的實測):高度偏離下滑線 >25 m 就重發航點,
#    而重發會 set_target_altitude_current() 把高度需求重錨到當下高度
#    ⇒ 每 RESEND_DT 秒抹掉一次累積的下降需求。離環到交接的 1950 m 之間
#    六機**淨爬升 +54~+175 m**。3000 讓重發條件恆為假 = 真正的單發鎖存。
#
# ⇒ 三處(本檔、run_west.sh、swarm_ros2_up.sh)現在一致,「真相源」這句話才成立。
LINE_TOL="${LINE_TOL:-3000.0}"
HDEM_TC="${HDEM_TC:-0.5}"
RESEND_DT="${RESEND_DT:-2.0}"
# ⚠️ 飛行記錄不要放 /tmp:WSL 一重啟就整個清空(2026-07-27 實測被清掉一整輪的
#    tot_*.csv / trial_*.log,離線分析全部做不了)。預設改寫到 ~/fixwing_ws/runs/。
# ── 幾何與速度包絡 ────────────────────────────────────────────────────
# ⚠️ 這些以前是寫死的。換成高速機體(jet_design.py 改裝)之後**每一項都必須跟著
#    放大**,因為它們全部是「距離」形式的時間常數或幾何量:
#    迴轉半徑 ∝ V²,70 m/s → 136 m/s 是 3.8 倍。不放大的話症狀是靜默的 ——
#    飛機被捕獲進一個做不到的圓、或 PN 只剩幾秒可用。
R_FAN="${R_FAN:-4000.0}"          # 扇形半徑;必須 > pushover_r
HALF_ANGLE="${HALF_ANGLE:-25.0}"
VMIN="${VMIN:-45.0}"              # 環上等待速度(要遠高於失速,又要飛得住)
VMAX="${VMAX:-100.0}"             # 巡航規劃速度上限
MARGIN="${MARGIN:-40.0}"
REACH="${REACH:-260.0}"           # 認定「已抵 slot」的半徑
LAYER_M="${LAYER_M:-25.0}"        # 對齊階段的高度分層
PERCH_REACH="${PERCH_REACH:-260.0}"
IMPACT_REACH="${IMPACT_REACH:-25.0}"
OVERSHOOT="${OVERSHOOT:-300.0}"   # 穿越瞄準點放在 T 後方多遠
SWITCH_R="${SWITCH_R:-300.0}"
DIVE_SPEED="${DIVE_SPEED:-140.0}" # 俯衝指令空速上限(要高於機體極限才會壓機頭)
DIVE_RADIUS="${DIVE_RADIUS:-5.0}"
# 終端垂直分層(防撞);拉大可增加間隔。沿用舊的 DIVE_LAYER 名稱以免既有呼叫失效。
DIVE_LAYER_M="${DIVE_LAYER_M:-${DIVE_LAYER:-4.0}}"
# ⚠️ jet_rescatter.py 的引數是 **hold 秒數**,不是間距。四散的落點是寫死在該檔的
#    OFF 表裡(目標南方 3.6–4.4 km、橫向 ±1.3 km、高度 130 m),不吃參數。
#    (先前一度把它當成間距傳 1200,等於要它 hold 20 分鐘,整趟會直接逾時報廢。)
SCATTER_HOLD="${SCATTER_HOLD:-${SCATTER_M:-300}}"
# PN 油門律(見 tot_node._pn_cmd)。⚠️ pn_thr_trim 0.36 是抄舊機體的
# TRIM_THROTTLE 來的,換機體必須重設 —— 尤其用 THR_MAX 壓推重比之後,
# SET_ATTITUDE_TARGET 的 0..1 對應的是 THR_MIN..THR_MAX。
# ⚠️⚠️ GROUND_ALT 只是 auto_ring 在地形回報進來**之前**的暫用值。真值由 tot_node 的
#    _reground() 用實測地形重算。以前這裡寫死 −48,而 LOC 南移 100 km 後 HOME 落在
#    中央山脈(1839 m AMSL)、紅心在 −30 m ⇒ 真實 z_g = −1869,差了 1821 m。
#    後果是環半徑照「要掉 3100 m」算、飛機實際要掉 4969 m,殘餘正好 1821 m ——
#    整批「從紅心上空 1500–2300 m 飛過」的失敗都是這一條,而且偽裝成導引律問題。
# ⚠️ ROS 2 的 -p 會把 "200" 判成 INTEGER,而參數宣告是 DOUBLE ⇒ 節點啟動即崩
#    (InvalidParameterTypeException)。所有 float 參數一律自動補小數點,
#    不要靠呼叫端記得寫 200.0 —— 這個陷阱已經踩過兩次。
_f() { case "$1" in *.*|*e*|*E*) printf "%s" "$1";; *) printf "%s.0" "$1";; esac; }
DIVE_ANGLE_MAX="$(_f "${DIVE_ANGLE_MAX:-0.0}")"   # 放行的幾何守門員(度;0=關)
PERCH_STEP="$(_f "${PERCH_STEP:-0.0}")"    # 高度通道協同:各機 perch 遞增量(m)
DIVE_FULL_THR="${DIVE_FULL_THR:-False}"    # 終端鎖全油門(需 PERCH_STEP>0)
# ⭐ 俯衝段的 THR_MAX。PN 送的 thr=1.0 會被 servos.cpp 夾到 THR_MAX,
#    巡航設計值 25 之下「全油門」只有 25%。0 = 不斷言(沿用巡航值)。
DIVE_THR_MAX="${DIVE_THR_MAX:-100.0}"
STAGGER_DEPART="${STAGGER_DEPART:-False}"  # 分時獨立解脫(見 tot_node;預設關)
STAGGER_LEAD="$(_f "${STAGGER_LEAD:-0.0}")"
WEAVE_PERIOD="$(_f "${WEAVE_PERIOD:-22.0}")"   # ALIGN 的 S 型換向週期
PN_LOS_FLOOR="$(_f "${PN_LOS_FLOOR:-0.0}")"    # 幾何前饋:指令俯角不准比視線淺
# ── 動態 PN 交棒(2026-08-06)────────────────────────────────────────────
# gate 不再是寫死的距離,而是由「PN 段被要求飛多陡」反推:
#   R_turn = V²/(g(cosγ − n_z));d_min = R_turn·sinθ + (h − R_turn(1−cosθ))/tanθ
#   gate   = clamp(d_min/Φ, 下限, 上限, 環半徑)
# ⚠️ 為什麼不是綁環半徑的固定比例:828 架次全掃,比值 f 對殘餘高度的 Spearman
#    只有 +0.103,交棒視線俯角 γ 是 +0.892。綁 f 是錯的參數化。
# ⚠️ GATE_THETA_MAX=0 ⇒ 完全退回絕對 GATE_R(預設就是 0,所以預設零行為變化)。
#    實測 PN 只交得出 23.3–33.9°(三趟六架次),要開的話 35 是有依據的上緣。
GATE_THETA_MAX="$(_f "${GATE_THETA_MAX:-0.0}")"
GATE_PHI="$(_f "${GATE_PHI:-0.80}")"
GATE_R_MIN="$(_f "${GATE_R_MIN:-2500.0}")"
GATE_R_MAX="$(_f "${GATE_R_MAX:-6000.0}")"
GATE_HOLD_N="${GATE_HOLD_N:-3}"
PN_NZ_MIN="$(_f "${PN_NZ_MIN:-0.05}")"         # PN 法向過載下限;陡俯角放負值
DIVE_PITCH_MAX="$(_f "${DIVE_PITCH_MAX:-45.0}")"  # PN 抬頭上限
PERCH_CLIMB_V="$(_f "${PERCH_CLIMB_V:-0.0}")"     # PERCH 爬升空速(0=沿用預設)
# ⚠️⚠️ 預設**必須是負值**。tot_node 的放開條件是 `if dive_pitch_min < 0.0:`,
#    設 0.0 等於讓整段「放開低頭/抬頭權限」的程式碼永遠不執行,
#    PTCH_LIM_MIN_DEG 會停在巡航段的 −20 ⇒ 實際航跡角上限只有 ~16°。
#    實測 DV1:指令 27° 俯衝,ATT.DesPitch 全程最小 −20.0°,六機從紅心
#    上空 2.3–2.7 km 平飛掠過,而那 218–228 m/s 是平飛頂速不是俯衝速度。
DIVE_PITCH_MIN="$(_f "${DIVE_PITCH_MIN:--80.0}")"  # 進俯衝才放開的低頭上限(負值)
PN_THR_TRIM="$(_f "${PN_THR_TRIM:-0.36}")"
PN_THR_GAIN="${PN_THR_GAIN:-0.030}"
PN_LATE_FULL="${PN_LATE_FULL:-True}"
# ── miss 出口(tot_node 的 M1/M2)。以前全部靠節點端的預設值,航次組態改了
#    也調不動,而其中一個是**會整趟報廢**的耦合門檻,必須看得見、調得動。
# ⚠️⚠️ MISS_FAR_M 與 pushover 環半徑耦合:ring = (perch_alt−dive_alt)/tan(dive_angle)
#    ⇒ dive_angle 25° 是 6.4 km、5° 是 34 km;perch 6000 m + 15° 是 22.4 km。
#    環半徑 ≥ MISS_FAR_M 時,還在環上**合法等放行**的機會被判逃逸,而且是
#    全隊同時 ⇒ 整趟 0/6 結案,在總表上跟「飛行失敗」一模一樣,真因卻是門檻。
#    tot_node 在 dive_go 會自動把它夾到 MISS_FAR_K × max(環半徑, 當下最遠),
#    並印一行含「生效值」的 log(所以會進 summary.md)。MISS_FAR_K=0 關掉自動夾;
#    另有一道獨立保險:逃逸規則只對**已放行**的機檢查。
MISS_FAR_M="$(_f "${MISS_FAR_M:-20000.0}")"
MISS_FAR_K="$(_f "${MISS_FAR_K:-2.0}")"
MISS_WINDOW_S="$(_f "${MISS_WINDOW_S:-60.0}")"
MISS_SLOW_GS="$(_f "${MISS_SLOW_GS:-5.0}")"
MISS_SLOW_SEC="$(_f "${MISS_SLOW_SEC:-60.0}")"
# 命中鎖存後仍續錄這麼多秒才印摘要(收尾等待)。
# ⚠️ 這是**續錄**秒數,不是量測窗:dmin / hit_alt 在各機鎖存當下就凍了,
#    調它不會改變摘要行的任何數字,只會多錄/少錄幾秒 CSV。
IMPACT_SETTLE="$(_f "${IMPACT_SETTLE:-4.0}")"
# ── 最近水平距(dmin)沒有可調的收斂窗,這是刻意的 ────────────────────────
# ⚠️⚠️ 2026-08-04 移除 DMIN_HYST_M / DMIN_HOLD_S(節點端的 dmin_hyst_m /
#    dmin_hold_s 一併移除)。那個遲滯窗要鎖存後 31.7–45.5 s 才收,而機體在
#    鎖存後 1.9–2.6 s 就觸海了 —— SITL 不終結觸海的機體,SW1 的 U3 掉到海面下
#    95.7 m 又爬回 +331 m、24.1 s 後再穿一次紅心。窗收進去的是**第二趟**:
#    U3 32.6→22.6(剛好從 MISS_MAX=30 的失敗側跨到通過側)、U5 217.9→67.1。
#    那是把指標改好看,不是改準。現在 dmin 一律鎖存當下凍結。
# ⚠️ 摘要行的「最近水平距」因此是首航次真最近點的**上界**(誤差 ≤ 一個取樣步,
#    實測 3–9 m)。要更細的量,用 stot_score.py 的 d1(點到線段內插 + 觸地收窗)。
OUT_DIR="${OUT_DIR:-$HOME/fixwing_ws/runs}"
mkdir -p "$OUT_DIR"
LOG="${LOG:-$OUT_DIR/trial_${TRIAL}.log}"
CSV_BASE="$OUT_DIR/tot_${TRIAL}"
# ⚠️⚠️ 指令記錄器以前寫死在 runs/cmdlog/uav{id}_cmd_telemetry.csv,而且是用 "w"
#    開檔 ⇒ **每一趟都把上一趟的證據覆蓋掉**。P91 的資料就是靠開跑前手動備份
#    才留下來的。這是「先量,不要猜」唯一的原始素材,不能只留最後一趟。
#    改成逐趟目錄;wingman_node._cmd_log_open 讀這個環境變數。
export CMD_LOG_DIR="${CMD_LOG_DIR:-$OUT_DIR/cmdlog/${TRIAL}}"
mkdir -p "$CMD_LOG_DIR"

[ -f "$CSV" ] || echo "trial,spread_s,max_miss_m,max_abs_alt_m,n_hit,pass,note" > "$CSV"

# ⚠️ 機數閘門要跟著 SWARM_COUNT。寫死 6 的話,兩機漸進測試會在這裡直接 exit 1。
_WANT_N="${SWARM_COUNT:-6}"
# ⚠️⚠️ 一次把機數 export 給**所有**下游 python 幫手。
#    scripts/wsl/ 底下有 10 支會逐機開連線的腳本(jet_catapult_launch / jet_prep /
#    jet_rescatter / jet_show_cruise / jet_show_ros2 / preflight_check /
#    tot_scatter / stot_covariate / stot_mission_anim / _show_anim),
#    它們各自讀 N 或 COUNT、預設都是 6。以前 stot_trial.sh 一個都沒傳,
#    所以兩機航次會在第一支就 ConnectionRefused(UAV3~6 的埠沒人聽)並中止,
#    而錯誤訊息長得像「彈射失敗」——真因是機數沒傳到。
#    在這裡 export 一次,勝過去改 10 支腳本、也勝過未來每加一支就漏一次。
export N="$_WANT_N" COUNT="$_WANT_N" SWARM_COUNT="$_WANT_N"
if [ "$(pgrep -x arduplane | wc -l)" -ne "$_WANT_N" ]; then
  echo "✗ SITL 不是 $_WANT_N 架(現在 $(pgrep -x arduplane | wc -l))—— 先跑 swarm_sitl_up.sh"
  exit 1
fi

echo "═══ TRIAL $TRIAL  PN=$PN pace=$DIVE_PACE layer=$DIVE_LAYER track=$TRACK_FRAC ═══"
# ⚠️⚠️ **開跑前把實際生效值讀回來驗證。** 本輪連續踩了 10 個靜默失敗,
#    bash -n / ast.parse / colcon build 一個都抓不到 —— 因為它們全是
#    「值合法但語意錯」或「送出去了但沒生效」。共同結構是我們相信自己
#    送出的值而不去讀回來。這一關擋在所有航次之前。
#    ⚠️ 它檢查的不只是單一值,更重要的是**參數之間的關係**
#      (catch_speed vs AIRSPEED_MAX、WP_LOITER_RAD vs reach、
#       pushover_r vs 最小迴轉半徑、DIVE_PITCH_MIN 的正負)——
#      壞掉的從來都是關係。
if ! COUNT="${SWARM_COUNT:-6}" REACH="$REACH" DIVE_ANGLE="$DIVE_ANGLE" PERCH_ALT="$PERCH_ALT"      R_FAN="$R_FAN" DIVE_PITCH_MIN="$DIVE_PITCH_MIN"      CATCH_SPEED="${CATCH_SPEED:-157.0}"      python3 "$SR/preflight_check.py"; then
  echo "✗ 開跑前驗證未通過 —— 中止(設 STRICT=0 可強制略過,但不建議)"
  exit 1
fi

# ═══ 素材優先於歸因(2026-08-05 blocker D)═══════════════════════════════
# ⚠️⚠️ 以前只有「正常結案」那一條路徑會呼叫 make_artifacts.sh。逾時 / 節點提前
#    死亡 / 彈射失敗全部 exit 掉 ⇒ **那一趟的原始素材永久消失**:在
#    stot_repeat.sh 之下每趟都重啟 SITL,dataflash .BIN 下一趟就被覆寫;
#    /tmp/trial_launch.log 根本不帶航次編號、只留最後一趟。
#    專案規定是「航次一結束**立刻**產素材」,而「結束」包含失敗的結束。
#    ⇒ 改成 EXIT trap:任何結束路徑(含 Ctrl-C / SIGTERM)都跑一次,且只跑一次。
# ⚠️ 裝在 SITL 檢查與 preflight **之後**:那兩關還沒飛,沒有素材可留;
#    起飛之後就算彈射失敗,/tmp/trial_launch.log 也是本趟唯一的彈射證據。
# ⚠️ make_artifacts.sh 每一項都自帶 `[ -f … ]`,餵半套資料只會少產幾項,
#    不會整批失敗 —— 所以「先產再說」是安全的。
# ⚠️ bash 的 EXIT trap 不會被 ( … ) 子殼繼承,底下那些背景子殼結束時不會誤觸。
# ⚠️ trap 函式**不可以自己 exit**,否則會蓋掉真正的退出碼(歸因就沒了)。
_ARTIFACTS_DONE=0
_artifacts_once() {
  [ "$_ARTIFACTS_DONE" = "1" ] && return 0
  _ARTIFACTS_DONE=1
  bash "$SR/make_artifacts.sh" "$TRIAL" || echo "[artifacts] 產出失敗(不影響航次結果)"
  return 0
}
trap _artifacts_once EXIT
trap '_artifacts_once; exit 130' INT TERM
# TOT_ONLY=1:蜂群已經在空中(上一趟沒打完就中止),只重跑 tot_node。
# 省掉重啟 SITL + 彈射 + 四散的 ~10 分鐘,迭代導引邏輯時用這個。
if [ "${TOT_ONLY:-0}" = "1" ]; then
  echo "[1/1] 蜂群已在空中,直接跑 tot_node"
else
echo "[1/3] 彈射起飛…"
# ⚠️ CLIMB_TO_ALT:ROS2 編隊模式一定要在**協同層接手之前**爬完巡航高度,
#    否則僚機每秒重發的航點會把高度目標重錨在當下高度、永遠爬不上去(見該腳本說明)。
# ⚠️⚠️ 沒設就直接擋下來,不要讓它「看起來有跑」。實測 Q1:CLIMB_TO_ALT 沒設 ⇒
#    只爬到 ~100 m 就交棒 ⇒ 階段 2 整段在爬升中跑,末端誤差 44 km。
#    那趟的數字被當成「編隊邏輯壞了」差點誤導整輪診斷 —— 上游沒過就往下量,
#    是這個專案代價最高的一類錯誤(見 docs 的方法論教訓)。
if [ "${CRUISE_MODE:-scatter}" = "ros2" ] \
   && [ "$(awk -v a="${CLIMB_TO_ALT:-0}" 'BEGIN{print (a<1000)?1:0}')" = "1" ]; then
  echo "✗ CRUISE_MODE=ros2 但 CLIMB_TO_ALT=${CLIMB_TO_ALT:-未設}(<1000 m)。"
  echo "  協同層接手前必須先爬到巡航高度,否則階段 2 量到的是爬升段的數字。"
  echo "  設 CLIMB_TO_ALT=\${SHOW_CRUISE_ALT:-3100} 再跑。"
  exit 1
fi
# ⚠️ 彈射間隔(使用者 2026-07-31 定調 3 s)。
#    3 s × 157 m/s = 471 m,而編隊縱向 slot 在 3277/6553/6000 m ——
#    六機離地後**都在自己 slot 的前方**,需要往後退而不是往前追。
#    這個方向是有利的:減速權限(−45 EAS)是加速權限(+21)的 2 倍,
#    落後機追不上的老問題在這個時序下不會發生。
#    (均勻 25.48 s 反而會讓 UAV6 落在 20 km 後、需要前追 14 km。)
# ⚠️⚠️ **註解不可以夾在 `\` 續行的中間。** 這幾行原本就放在 `timeout ... \`
#    和 `python3 ...` 之間,而 `\`+換行會把註解行接上來 ⇒ 實際執行的是
#    `timeout "260"`(沒有帶命令,報 "Try 'timeout --help'"),然後 python3
#    變成獨立的一行、**完全沒有 timeout 保護**。整段靜默失敗,沒有錯誤退出碼。
# ⚠️ 第二個參數是 climb_wait(**秒**),不是高度。爬到巡航高度是由
#    CLIMB_TO_ALT 環境變數驅動的(見 jet_catapult_launch.py)——
#    沒設就等於跳過「協同層接手前先爬完」這一步,實測 Q1 因此只爬到 ~100 m,
#    整趟階段 2 都在爬升中跑,末端誤差 44 km(和導引邏輯完全無關)。
# ⚠️⚠️ LAUNCH_TIMEOUT 從 260 放大到 900。260 是在上面那個續行 bug **還在**的
#    時候訂的 —— 那時 timeout 根本沒有生效,所以再小也無所謂。bug 修好之後
#    它第一次真的開始計時,實測 S1 在 2347 m 就被砍掉(爬到 3100 m 要 ~370 s
#    ,加上彈射序列與 climb_wait 約需 500 s)。
#    **修好一個靜默失效的保護之後,要重新檢查它的參數還合不合理。**
( cd "$SR" && STAGGER_GEOM="${STAGGER_GEOM:-0}" \
    V_SPACING="${V_SPACING:-4000.0}" FORM_V_GS="${FORM_V_GS:-157.0}" \
    CLIMB_THR_MAX="${CLIMB_THR_MAX:-0}" \
    CLIMB_TO_ALT="${CLIMB_TO_ALT:-0}" \
    CLIMB_DONE_V="${CLIMB_DONE_V:-$(awk -v g="${FORM_V_GS:-157.0}" \
        'BEGIN{printf "%.0f", g*0.857}')}" \
    timeout "${LAUNCH_TIMEOUT:-900}" \
    python3 jet_catapult_launch.py "${STAGGER:-3}" "${CLIMB_WAIT:-90}" ) \
    > /tmp/trial_launch.log 2>&1
_launch_rc=$?
if ! grep -q "全員 GUIDED" /tmp/trial_launch.log; then
  echo "$TRIAL,,,,0,0,launch_failed" >> "$CSV"; echo "  ✗ 彈射失敗"; tail -5 /tmp/trial_launch.log; exit 1
fi
# ⚠️⚠️ 「全員 GUIDED」印在**爬升之前**(jet_catapult_launch.py:259),所以它
#    只證明「都進 GUIDED 了」,**不證明爬到巡航高度**。Q1 事故就是這樣:
#    只爬到 ~100 m 卻通過閘門,末端誤差 44 km;當時補的閘門(:246)只檢查
#    CLIMB_TO_ALT 這個**變數有沒有設**,漏的是「有沒有真的爬到」。
#    2026-08-06 補上兩道:python 端爬不完會非零退出 + 印「✗ 爬升未完成」;
#    這裡兩種都認,少一種都會被漏掉。
if [ "${CLIMB_TO_ALT:-0}" != "0" ]; then
  if [ "$_launch_rc" != "0" ] || grep -q "✗ 爬升未完成" /tmp/trial_launch.log \
     || ! grep -q "全員抵巡航高度" /tmp/trial_launch.log; then
    echo "$TRIAL,,,,0,0,climb_failed" >> "$CSV"
    echo "  ✗ 爬升未完成(rc=$_launch_rc)—— 不繼續,後面整段會失去意義"
    grep -E "climb→|全員抵巡航高度|✗ 爬升未完成" /tmp/trial_launch.log | tail -6
    exit 1
  fi
fi
# CRUISE_MODE=formation:走 jet_show_cruise.py 的「V 字編隊長程巡航 → 四散」,
#   也就是表演真正要看的那一段(FORMUP → CRUISE → SCATTER)。
#   預設 scatter 是 jet_rescatter.py:各機直接飛到六個固定點繞圈待命,快但**沒有編隊**。
# ⚠️ 編隊巡航要飛到南方的起跑線再北上,航程長很多,timeout 要跟著放大。
if [ "${CRUISE_MODE:-scatter}" = "ros2" ]; then
  # ⭐ ROS2 編隊層負責隊形(wingman_node),本層只驅動長機沿航線前進。
  #    分工照 docs/swarm_architecture.md:① SITL 飛控、② ROS2 協同。
  echo "[2/3] ROS2 編隊層長程巡航(隊形 ${SHAPE:-V})…"
  # ⚠️ 編隊高度一定要傳。不傳的話僚機用節點預設 50 m,長機卻爬到巡航高度 ——
  #    實測六機散在 10 km、一架落到地面(N1)。
  # ⚠️⚠️ **速度也一定要傳,而且這一條比高度更隱蔽。** swarm_ros2_up.sh 沒有 PRESET
  #    時落到泡棉機的預設:nominal 14 / catch 20 / wait 10 m/s,而本機體長機巡航
  #    75 m/s ⇒ 僚機被命令用 1/4 的速度飛,**物理上不可能跟上**,間距只會單調拉開。
  #    實測 AG1(ROS2 V 編隊):僚機相對長機的橫向偏移全程 0–60 m(設計是 ±287/±574),
  #    縱向從 926 m 一路拉到 **13688 m** —— 看起來是「一縱列」,其實是編隊從未成形。
  #    catch_radius 400 同樣致命:落後超過 400 m 就吃 catch_speed,而那是 20 m/s,
  #    落後 13 km 也只給 20 m/s。放到 3000 讓速度斜坡覆蓋真實的誤差量級。
  #    ⚠️ formed_radius 必須 < V_SPACING(見 launch 檔註解),否則「在 slot 250 m 內
  #    就算到位」會讓隊形永遠收不緊。
  #
  # ⚠️⚠️ **追趕速度必須由巡航速度推導,不可以綁 SHOW_VMAX。** 這兩個是不同的量:
  #    SHOW_VMAX 是「長機航線腳本的速度上限」,catch_speed 是「僚機落後時要多快」。
  #    先前把 CATCH_SPEED 預設成 ${SHOW_VMAX:-105},而巡航是 157 ——
  #    僚機一落後就被命令**減速到 105**,間距只能單調拉開。比 nominal=14 更隱蔽,
  #    因為 105 看起來是個合理的數字。以下一律從 FORM_V 推。
  #
  # ⚠️ catch_radius 不是「多遠才開始追」,它是**迴路增益的倒數**:
  #    geodesy.speed_for_error 的斜坡從 err=0 起算、分母就是 catch_r,
  #    ⇒ 閉合率 = Δ·(err/catch_r),時間常數 τ = catch_r/Δ。
  #    先前的 3000 給 τ = 103 s,迴路是增益受限而非推力受限,誤差 422 m 在 30 s
  #    只收斂 24%。設在「可容忍落後量」的量級才會變成推力受限。
  #
  # ⚠️ WP_LEAD 在這條路徑上從來沒傳過(swarm_ros2_up.sh 預設 0.0,只有 PRESET=max
  #    給 3.5)。導引有效滯後 ~3.5 s,在 183 m/s 地速之下是 **640 m 的穩態誤差地板**,
  #    大於任何合理的 formed_radius ⇒ 隊形永遠判不出「成形」,間距推導全部不成立。
  #
  # ⚠️⚠️ **速度指令是空速,幾何是地速,兩者在 3100 m 差 1.167 倍。**
  #    DO_CHANGE_SPEED / nominal_speed / catch_speed 吃的都是空速(EAS),
  #    而間距、追趕距離、L_max 全是地面距離。混用會系統性差 17%。
  #    σ(3100 m) = 0.7345 ⇒ 空速 = 地速 × √σ = 地速 × 0.857。
  #    使用者 2026-07-29 定調:157 m/s 是**地速**。
  FORM_V_GS="${FORM_V_GS:-${SHOW_CRUISE_GS:-157}}"          # 地速(幾何用)
  FORM_V="${FORM_V:-$(awk -v g="$FORM_V_GS" 'BEGIN{printf "%.0f", g*0.857}')}"  # 空速(指令用)
  FORM_CATCH="${FORM_CATCH:-$(awk -v v="$FORM_V" 'BEGIN{printf "%.0f", v*1.16}')}"
  # ⚠️⚠️ 等待速度**不可以**取巡航的對稱下緣。速度權限是不對稱的:
  #    加速靠剩餘推力(在 135 EAS 只有 +1.33 m/s²),減速靠阻力(−2.62 m/s²,快 2 倍),
  #    而可用區間也不對稱 —— 加速側到 156 EAS(+21),減速側到 ~90 EAS(**−45**)。
  #    取 0.84 倍(113)只用掉一半的減速權限,max-consensus 在大落後時就退化成
  #    和舊的二值切換一樣(都夾在同一個地板上,閉合率同為 22 m/s)。
  #    取 0.67 倍(90 EAS)⇒ 閉合率 157−90 = **67 m/s,是原本的 3 倍**。
  #    90 EAS 遠高於失速(AIRSPEED_MIN 25),且剩餘推力 61 N ⇒ 爬升餘裕充足。
  FORM_WAIT="${FORM_WAIT:-$(awk -v v="$FORM_V" 'BEGIN{printf "%.0f", v*0.67}')}"
  # 可容忍落後量 L_max ≈ Δ_地速 · T_recover,T_recover 取 30 s;但本機體是
  # **加速度受限**不是速度受限(速度差衰減時間常數 ~19 s、初始加速度僅 1.2 m/s²),
  # 所以再打 0.7 折。Δ 用地速算,因為它要換算成地面距離。
  FORM_LMAX="$(awk -v g="$FORM_V_GS" 'BEGIN{printf "%.0f", g*0.16*30*0.7}')"
  # ⚠️⚠️ **間距與控制門檻要分開推。** 這兩者以前綁在一起(V_SPACING = 2×L_max),
  #    那在「間距由控制律決定」的緊密編隊時代是對的;B-500 的松耦合觀測編隊
  #    間距是**任務需求**(觀測涵蓋寬度),不能再由追趕權限反推。
  #      幾何(任務決定):V_SPACING、V_ANGLE、V_TAIL
  #      門檻(速度權限決定):CATCH_RADIUS、CLOSE_TIME
  #      到位判準(雜訊量級決定):FORMED_RADIUS
  #    綁在一起的話,把間距拉到 4 km 會連帶把 catch_radius 拉到 4 km ——
  #    速度斜坡的比例區被撐爆,僚機在整個收斂過程都是全速追、沒有比例控制。
  V_SPACING="${V_SPACING:-4000}"          # 2.49 英里;區間 3218–4828
  V_ANGLE="${V_ANGLE:-35}"                # rank2 側向 ±4589 m ⇒ 涵蓋寬度 9.2 km
  # UAV6(備長)在 V 尾。⚠️ 這個值以前固定 220 m,而 rank2 已在 8 km 外 ——
  #    備長會擠在長機屁股後面,既不是編隊的一部分,接管時也離重心太遠。
  #    取 1.5×間距,排在 rank2 之後。
  V_TAIL="${V_TAIL:-$(awk -v s="$V_SPACING" 'BEGIN{printf "%.0f", s*1.5}')}"
  # catch_radius = 速度斜坡的比例區寬度,由**閉合迴路的時間常數**決定,
  # 與間距無關:τ_loop = catch_r / Δ。L_max(≈528)給 τ ≈ 24 s。
  FORM_CATCH_R="${FORM_CATCH_R:-$FORM_LMAX}"
  # formed_radius = 「已進入槽位」的判準,由**動態雜訊量級**決定(GPS + WP 滯後
  # + 陣風),不是由間距等比例縮放。3 英里間距下取 200 m(= 間距的 5%)。
  # ⚠️ 設太小(如 20 m)會讓 formed 永遠不成立,長機一路停在 wait_speed。
  FORM_FORMED_R="${FORM_FORMED_R:-200}"
  echo "[編隊] 地速 ${FORM_V_GS}(= 空速 ${FORM_V})/ 追趕 ${FORM_CATCH} /" \
       "等待 ${FORM_WAIT} m/s"
  echo "[編隊] 幾何(任務):間距 ${V_SPACING} m、半角 ${V_ANGLE}°、V尾 ${V_TAIL} m" \
       "⇒ 涵蓋寬度 $(awk -v s="$V_SPACING" -v a="$V_ANGLE" \
          'BEGIN{printf "%.1f", 4*s*sin(a*3.14159265/180)/1000}') km、" \
       "縱深 $(awk -v s="$V_SPACING" 'BEGIN{printf "%.1f", 2*s/1000}') km"
  echo "[編隊] 門檻(控制):catch_r ${FORM_CATCH_R}、formed_r ${FORM_FORMED_R}、" \
       "close_time ${CLOSE_TIME:-60}s"
  ( cd "$SR" && MANUAL=1 PATROL=0 SHAPE="${SHAPE:-V}" \
      CONE_HALF="${CONE_HALF:-30.0}" \
      FORM_ALT="${SHOW_CRUISE_ALT:-3100}" PATROL_ALT="${SHOW_CRUISE_ALT:-3100}" \
      LAYER_SEP="${LAYER_SEP:-40}" \
      V_SPACING="$V_SPACING" V_ANGLE="$V_ANGLE" V_TAIL="$V_TAIL" \
      NOMINAL_SPEED="${NOMINAL_SPEED:-$FORM_V}" \
      CATCH_SPEED="${CATCH_SPEED:-$FORM_CATCH}" \
      WAIT_SPEED="${WAIT_SPEED:-$FORM_WAIT}" \
      FORMED_RADIUS="${FORMED_RADIUS:-$FORM_FORMED_R}" \
      CATCH_RADIUS="${CATCH_RADIUS:-$FORM_CATCH_R}" \
      CLOSE_TIME="${CLOSE_TIME:-60.0}" \
      CLIMB_GATE_HI="${CLIMB_GATE_HI:-1000}" CLIMB_CAP_LO="${CLIMB_CAP_LO:-1.5}" \
      CRUISE_ALT_THRESHOLD="${CRUISE_ALT_THRESHOLD:-3000.0}" \
      CRUISE_GATE_HYST="${CRUISE_GATE_HYST:-150.0}" \
      CRUISE_GATE_DWELL="${CRUISE_GATE_DWELL:-5.0}" \
      AIRSPEED_MAX_CAP="${AIRSPEED_MAX_CAP:-165.0}" \
      L1_LOOKAHEAD="${L1_LOOKAHEAD:-1500.0}" \
      CRUISE_ROLL_LIMIT="${CRUISE_ROLL_LIMIT:-0.0}" \
      ROLL_LIMIT_RESTORE="${ROLL_LIMIT_RESTORE:-${DIVE_ROLL_MAX:-80.0}}" \
      SPEED_DIRECT="${SPEED_DIRECT:-1}" \
      WP_LEAD="${WP_LEAD:-4.0}" \
      WP_INTERVAL="${WP_INTERVAL:-1.0}" \
      WP_MOVE_THRESH="${WP_MOVE_THRESH:-40}" \
      TOT_ENABLE=0 \
      bash "$SR/swarm_ros2_up.sh" ) > /tmp/trial_ros2.log 2>&1 &
  # ⚠️⚠️ TOT_ENABLE=0 不可拿掉,而且不是「順手關掉沒用的東西」——
  #    2026-08-05 起 swarm_ros2_up.sh 會一併拉起一個 **STANDBY tot_node**
  #    (那是給「AeroPlan 右鍵指定紅心」那條互動路徑用的)。批次航次這條路徑
  #    自己會在後面啟動航次用的 tot_node,兩個都起就會撞上分流埠的
  #    **先連者得**:MAVProxy 的 `--out tcpin` 只餵**第一個**連上的 client。
  #    待命節點先連上 6960/6970 ⇒ 航次節點是第二個 ⇒ 它一筆
  #    GLOBAL_POSITION_INT 都收不到,而且 `pkill -x tot_node` 殺掉待命節點
  #    之後那條 socket 會停在 **CLOSE-WAIT**,仍然佔著第一 client 的名額。
  #    症狀(KIL1,2026-08-06 實測):節點每 30 s 印
  #      「⚠️ 等機體定位中:0/2 台有 GLOBAL_POSITION_INT,缺 [1, 2]」
  #    然後整趟逾時,tot_*.csv 只有表頭 70 bytes。
  #    ⚠️ 這是**靜默失敗**:SITL 正常、MAVROS 正常、log 沒有任何 ERROR 指向根因。
  sleep 45                      # 等 MAVROS + 節點起來並接手 GUIDED
  ( cd "$SR" && timeout "${CRUISE_TIMEOUT:-900}" python3 jet_show_ros2.py ) \
      > /tmp/trial_scatter.log 2>&1
  _cruise_rc=$?
  bash "$SR/swarm_ros2_down.sh" >/dev/null 2>&1   # 交棒前收掉編隊層,避免和 tot_node 搶控制
  sleep 3
elif [ "${CRUISE_MODE:-scatter}" = "formation" ]; then
  echo "[2/3] V 字編隊長程巡航 → 四散…"
  ( cd "$SR" && timeout "${CRUISE_TIMEOUT:-900}" \
      python3 jet_show_cruise.py "${SHOW_CRUISE_V:-70}" "${SHOW_SPACING:-300}" ) \
      > /tmp/trial_scatter.log 2>&1
  _cruise_rc=$?
else
  echo "[2/3] 四散進扇形…"
  ( cd "$SR" && timeout "${CRUISE_TIMEOUT:-420}" \
      python3 jet_rescatter.py "$SCATTER_HOLD" ) > /tmp/trial_scatter.log 2>&1
  _cruise_rc=$?
fi
# ⚠️⚠️ 巡航段的 rc 先前**三個分支全部丟掉**,這正是下面那個 bug 能藏兩個月的原因:
#    jet_show_ros2.py 的 N 寫死 6 ⇒ 兩機航次連 UAV3–6 失敗、ConnectionRefusedError
#    當場崩掉 ⇒ 巡航段整個沒跑,而航次照常往下走。實測後果:TWO1/KIL2/KIL3 三趟的
#    「巡航編隊」從未執行,tot_node 的 ALIGN 跑了 963 s 把機隊從正北航點帶去扇形。
# ⚠️ 這裡刻意**只警告不中止**:巡航失敗時 ALIGN 仍能把機隊帶到扇形(三趟都證明了),
#    中止反而拿不到終端資料。但它必須**留下痕跡**,而且進 KPI,不能再靜默。
if [ "${_cruise_rc:-0}" != "0" ]; then
  echo "  ⚠️ 巡航段以 rc=${_cruise_rc} 結束(timeout 是 124)—— 本趟的『巡航編隊』沒有成立。"
  echo "     後面的 ALIGN 仍會把機隊帶到扇形,終端資料可用,但門檻②不可採信。"
  sed -n '$p;$!d' /tmp/trial_scatter.log 2>/dev/null
  grep -E "Error|Traceback|Refused" /tmp/trial_scatter.log 2>/dev/null | tail -3
  echo "$TRIAL,cruise_rc=${_cruise_rc}" >> "${CRUISE_RC_LOG:-/tmp/stot_cruise_rc.csv}"
fi
fi

echo "[3/3] STOT + 俯衝 → $LOG"
source /opt/ros/humble/setup.bash
source "$HOME/fixwing_ws/install/setup.bash" 2>/dev/null
setsid nohup ros2 run sar_swarm tot_node --ros-args \
  -p n:="${SWARM_COUNT:-6}" \
  -p tlat:="${TGT_LAT:-25.1406}" -p tlon:="${TGT_LON:-121.2325}" \
  -p talt:="$TALT" -p approach_from:="${APPROACH_FROM:-180.0}" \
  -p r_fan:="$R_FAN" -p half_angle:="$HALF_ANGLE" -p vmin:="$VMIN" -p vmax:="$VMAX" \
  -p plan_frac:=0.9 -p margin:="$MARGIN" -p perch_alt:="$PERCH_ALT" \
  -p ground_alt:="${GROUND_ALT:-0.0}" \
  -p dive_alt:=0.0 -p dive_layer:="$DIVE_LAYER_M" -p dive_angle:="$DIVE_ANGLE" \
  -p auto_ring:=True -p perch_alt_gate:=0.9 -p align_tol:="$ALIGN_TOL" \
  -p track_frac:="$TRACK_FRAC" -p dive_radius:="$DIVE_RADIUS" -p dive_pace:="$DIVE_PACE" \
  -p dive_spdweight:="$DIVE_SPDW" -p overshoot:="$OVERSHOOT" -p switch_r:="$SWITCH_R" \
  -p dive_speed:="$DIVE_SPEED" -p reach:="$REACH" -p layer_m:="$LAYER_M" \
  -p perch_reach:="$PERCH_REACH" \
  -p impact_reach:="$IMPACT_REACH" -p line_tol:="$LINE_TOL" -p resend_min_dt:="$RESEND_DT" \
  -p impact_alt_ok:="${IMPACT_ALT_OK:-30.0}" \
  -p impact_settle:="$IMPACT_SETTLE" \
  -p miss_far_m:="$MISS_FAR_M" -p miss_far_k:="$MISS_FAR_K" \
  -p miss_window_s:="$MISS_WINDOW_S" \
  -p miss_slow_gs:="$MISS_SLOW_GS" -p miss_slow_sec:="$MISS_SLOW_SEC" \
  -p target_amsl:="${TARGET_AMSL:-.nan}" \
  `# ⚠️ 預設值必須寫 YAML 的 .nan;寫成 nan 會被解析成字串,節點型別檢查直接掛。` \
  -p perch_climb_v:="$PERCH_CLIMB_V" -p dive_pitch_min:="$DIVE_PITCH_MIN" \
  -p dive_angle_max:="$DIVE_ANGLE_MAX" -p pn_los_floor:="$PN_LOS_FLOOR" \
  -p pn_nz_min:="$PN_NZ_MIN" -p dive_pitch_max:="$DIVE_PITCH_MAX" \
  -p weave_period:="$WEAVE_PERIOD" \
  -p stagger_depart:="$STAGGER_DEPART" -p stagger_lead:="$STAGGER_LEAD" \
  -p perch_step:="$PERCH_STEP" -p dive_full_thr:="$DIVE_FULL_THR" \
  -p dive_thr_max:="$DIVE_THR_MAX" \
  -p pn_thr_trim:="$PN_THR_TRIM" -p pn_thr_gain:="$PN_THR_GAIN" \
  -p pn_late_full:="$PN_LATE_FULL" \
  -p dive_hdem_tconst:="$HDEM_TC" \
  -p perch_band:="$PERCH_BAND" -p perch_settle:="$PERCH_SETTLE" \
  -p kill_on_touchdown:="$KILL_ON_TOUCHDOWN" -p kill_agl_m:="$KILL_AGL_M" \
  -p kill_confirm_n:="$KILL_CONFIRM_N" \
  -p hold_radius:="$HOLD_RADIUS" -p goto_radius:="$GOTO_RADIUS" \
  -p pn_enable:="$PN" -p gate_r:="$GATE_R" \
  -p gate_theta_max:="$GATE_THETA_MAX" -p gate_phi:="$GATE_PHI" \
  -p gate_r_min:="$GATE_R_MIN" -p gate_r_max:="$GATE_R_MAX" \
  -p gate_hold_n:="$GATE_HOLD_N" \
  -p pn_n_lat:="$N_LAT" -p pn_n_vert:="$N_VERT" -p pn_lead:="$PN_LEAD" \
  -p pn_vmax:="$PN_VMAX" -p pn_freeze_r:="$FREEZE_R" \
  -p pn_freeze_tgo:="$FREEZE_TGO" \
  -p gate_profile:="$GATE_PROFILE" -p gate_h:="$GATE_H" \
  -p csv:="$CSV_BASE" \
  -p link_base:=6960 -p link_stride:=10 > "$LOG" 2>&1 < /dev/null &
TOT_PID=$!
# ⚠️⚠️ 記下**自己這一趟**的行程。先前這裡沒記,收尾用 `pkill -x tot_node`
#    是全機無差別 —— 實測 D451 的收尾把正在俯衝的 R451 節點殺在 t=755 s,
#    而 R451 預定共同命中在 t=757.7,**差 2.5 秒**;R451 自己的預算還剩 51%
#    沒用到,卻被記成 "timeout / 觸地 0/6"。
#    ⇒ 那一趟的所有終端數字在方法學上不可歸因於飛行失敗。
#    平行跑兩趟時,無差別 pkill 會讓**後啟動的那趟殺掉前一趟**,而且靜默。
#
# ⚠️⚠️ 2026-08-05:$! **不是節點的 PID,是 `ros2 run` 這顆 CLI wrapper 的 PID。**
#    /opt/ros/humble/lib/python3.10/site-packages/ros2run/api/__init__.py 的
#    run_executable() 是 `subprocess.Popen(cmd)`(:64)+ `process.communicate()`
#    (:67)⇒ **節點是 wrapper 的子行程**;而 setsid 在自己還不是行程組長時
#    不 fork,所以 $! 就停在 wrapper 上(實測 $! == wrapper 自報 PID == PGID == SID)。
#    只 `kill $TOT_PID` 的話 wrapper 死了、**tot_node 還活著**,會跟下一趟搶控制
#    —— 這是無差別 pkill 那個坑的反面版本,而且更隱蔽(沒有任何錯誤訊息)。
#    ⇒ 收尾殺**行程群組**:setsid 已經把這一趟關進自己的 session/PGID,
#      負號 PID 只涵蓋本趟的所有子孫,兼顧「收得掉節點」與「不誤殺平行趟次」。
# ⚠️ PGID **讀回來**,不要假設它等於 $!(本專案的鐵則:不要相信自己送出的值)。
#    讀不到就退回用 $!,至少不會比舊寫法差。
TOT_PGID="$(ps -o pgid= -p "$TOT_PID" 2>/dev/null | tr -d ' \t')"
case "$TOT_PGID" in ''|*[!0-9]*) TOT_PGID="$TOT_PID";; esac
echo "[trial] ros2 run wrapper PID=$TOT_PID / 行程群組 PGID=$TOT_PGID" \
     "(收尾殺整個群組 —— 節點是 wrapper 的子行程,殺 wrapper 收不掉它)"

# ⚠️ 高速機體要放寬:T/W 0.274 的最佳爬升率只有 ~13.8 m/s,爬到 perch 2900 m
#    就要 ~200 s,再加上 11 km 扇形的進場與 49 s 俯衝,15 分鐘會不夠。
# ⚠️⚠️ 75 tick × 12 s = **剛好 900 s = 15 分鐘** —— 註解說「15 分鐘會不夠」,
#    而預設值就是 15 分鐘。實測 CA1 被砍在 t+900 s:
#        t+773s 全員抵扇形 → t+797s 全員就緒 → 「同步內切到環,預計 332s」
#    需要 ~1129 s。tot_node 收到的是 ExternalShutdownException(被外部砍),
#    不是墜機也不是導引失敗 —— 但總表只會記成「0/6 逾時」,看起來像飛行失敗。
# ⚠️ 抵扇形耗時取決於**上游隊形有多散**:階段 2 沒收好時僚機要多飛數十 km。
#    預算必須涵蓋「階段 2 不完美」的情況,否則終端結果無法歸因。
ok=0
# ⚠️⚠️ break 條件是「航次**結案了**」,不是「成功了」——成功與否由後面的
#    _kpi_parse.py 判。tot_node 的摘要行現在是**哨兵與達標描述分離**:
#      ★★ 俯衝結案:{命中}/{n} 命中、{miss} MISS[…];{達標描述}。{量測}
#    固定哨兵「★★ 俯衝結案:」在四種結案形狀下都存在:
#      (a) 零 miss 全部低空觸地  (b) 零 miss 但有機高空掠過(RD1 形狀)
#      (c) 有 miss 有命中        (d) 全部 miss
#    合約來源:sar_swarm_patches/tot_node.py 的 CLOSEOUT_SENTINEL。
#    ⚠️ 第二個 alternation 只為了讀 2026-08-05 以前的舊 log(當時沒有哨兵)。
#    ⚠️⚠️ 曾經只認達標描述:上一版「零 miss 但高空掠過」會印
#      「★★ 俯衝命中紅心:⚠️ 名義命中…」,兩個舊字串都不含 ⇒ 這個迴圈空等
#      1440 s、被記成 timeout,連 make_artifacts.sh 都跳過。RD1 就是這形狀。
#      SW1 則是另一種:有一架脫靶就永遠等到 WAIT_TICKS 用完,跑了 3.6 小時。
# ⚠️⚠️ 結案匹配式**只准有一條**(tests/test_summary_sentinel.py 直接從本檔原始碼
#    正規表示式萃取那一條偵測式,多一條就當萃取失敗 = 假迴歸鎖)。
#    ⇒ 包成函式,底下三個呼叫點共用同一條式子,不可能漂移;而且註解裡也不可以
#      再出現第二個看起來像偵測式的字樣(寫過一次,測試當場就紅了)。
_closed_out() { grep -qE "★★ 俯衝結案:|俯衝命中紅心:全員命中" "$LOG"; }
node_died=0
for w in $(seq 1 "${WAIT_TICKS:-120}"); do    # 每 tick 12 s ⇒ 預設 1440 s
  sleep 12
  # 先驗 log 再驗行程:節點可能在同一個 tick 內印完摘要就正常退出,
  # 順序顛倒的話「跑完了」會被記成「節點死了」。
  if _closed_out; then ok=1; break; fi
  # ⚠️⚠️ 2026-08-05 blocker D —— **第五種結案形狀:沒有結案行。**
  #    節點在 t=5 s 崩潰 / 被外部 kill / done 永遠到不了 n(_done_count 掃
  #    self.pos,某架既不 impact 也不 missed)時,舊迴圈只 grep $LOG、
  #    **完全不檢查 TOT_PID 是否還活著** —— 而 PID 就記在 30 行之上。
  #    後果:照樣空轉滿 WAIT_TICKS(預設 120×12 s = 1440 s)才寫
  #    `$TRIAL,,,,$nh,0,timeout`,與「飛得很慢的真逾時」在總表上一模一樣
  #    (歸因錯誤),而且以前還會跳過 make_artifacts.sh ⇒ 素材永久消失。
  if ! kill -0 "$TOT_PID" 2>/dev/null; then
    # grep 與 kill -0 之間仍有一個微小競態(剛好在這兩行之間印完摘要並退出)。
    # 判死之前再驗一次 log —— 寧可多讀一次,也不要把成功的航次記成 node_died。
    if _closed_out; then ok=1; else node_died=1; fi
    break
  fi
done
# ⚠️ 窄縫:done 達到 n 之後,節點還要**續錄 impact_settle 秒**才印摘要行。
#    WAIT_TICKS 剛好在這 4 s 內用完時,舊碼會寫出「nh=6 但 note=timeout」的
#    自相矛盾列 —— 六機都命中了,卻被歸因成「飛不完」。
#    給一段**有界**的寬限期(impact_settle + 8 s,上限 60 s),而且只在節點
#    還活著時等:等得到就走正常結案路徑,等不到才落到下面的 settle_cut。
if [ "$ok" -ne 1 ] && [ "$node_died" -ne 1 ]; then
  _grace="$(awk -v s="$IMPACT_SETTLE" \
      'BEGIN{g=s+8; if(g>60)g=60; if(g<1)g=1; printf "%d", g}')"
  echo "[trial] 預算用完但節點還活著 —— 寬限 ${_grace}s 等結案行" \
       "(impact_settle=$IMPACT_SETTLE;這是為了不把『已全員命中、只差摘要』" \
       "誤記成逾時)"
  for _g in $(seq 1 "$_grace"); do
    if _closed_out; then ok=1; break; fi
    if ! kill -0 "$TOT_PID" 2>/dev/null; then
      if _closed_out; then ok=1; else node_died=1; fi
      break
    fi
    sleep 1
  done
fi
# ⚠️ 只收自己這一趟的行程群組,絕不用無差別 `pkill -x tot_node`(見上方說明)。
#    先 TERM 讓 rclpy 走正常關閉(會 flush CSV),等 3 s 再 KILL 保底。
# ⚠️⚠️ 對象是**群組**不是 $TOT_PID:$TOT_PID 是 `ros2 run` wrapper,節點是它的
#    子行程。實測(同構複製品:python wrapper 用 Popen+communicate 起子行程、
#    setsid nohup 背景執行):
#      舊寫法 kill -TERM/-KILL $TOT_PID  ⇒ wrapper DEAD、**節點 ALIVE**(殘留 PID 還在)
#      新寫法 kill -TERM/-KILL -$TOT_PGID ⇒ wrapper DEAD、節點 DEAD,且節點確實
#                                           收到 SIGTERM(rclpy 才有機會 flush CSV)
#      平行兩趟時只殺趟次2的群組 ⇒ 趟次1 的節點不受影響(ALIVE/DEAD)
# ⚠️ 用 `kill -0 -$PGID` 判存活而不是 `kill -0 $TOT_PID`:wrapper 已被 bash 回收
#    (或被外部殺掉)但節點還在時,前者仍為真、後者會漏掉 —— 那正是最該收的情況。
_pg_alive() { kill -0 "-$1" 2>/dev/null; }
if [ -n "${TOT_PGID:-}" ] && _pg_alive "$TOT_PGID"; then
  kill -TERM "-$TOT_PGID" 2>/dev/null
  for _k in 1 2 3; do _pg_alive "$TOT_PGID" || break; sleep 1; done
  if _pg_alive "$TOT_PGID"; then kill -KILL "-$TOT_PGID" 2>/dev/null; sleep 1; fi
fi
# ⚠️ 收乾淨了沒有?**讀回來驗證**。殘留的節點會跟下一趟搶控制,而且完全靜默 ——
#    這正是本次迴歸的傷害形式,所以它必須看得見。
if [ -n "${TOT_PGID:-}" ] && _pg_alive "$TOT_PGID"; then
  echo "[trial] ⚠️⚠️ 行程群組 $TOT_PGID 在 TERM+KILL 之後仍有殘留 ——" \
       "下一趟會跟它搶控制:"
  ps -eo pid,ppid,pgid,stat,comm 2>/dev/null | awk -v g="$TOT_PGID" '$3==g' \
      | sed 's/^/    /'
  pkill -g "$TOT_PGID" -x tot_node 2>/dev/null   # 最後手段,仍只限本趟群組
fi
if [ "$ok" -ne 1 ]; then
  nh=$(grep -c "命中紅心 @" "$LOG")
  # ⚠️⚠️ 「沒有結案行」有三種成因,以前**全部寫成 timeout** —— 在總表上長得
  #    一模一樣,歸因整個錯掉(而歸因錯誤是本專案代價最高的一類錯誤)。
  #      node_died  節點提前死亡(崩潰 / 被外部 kill)。剩下的預算沒有用掉,
  #                 這一趟的終端數字在方法學上**不可歸因於飛行失敗**。
  #      settle_cut 六機都命中了,預算卻在 impact_settle 續錄窗內用完
  #                 (寬限期也沒等到)。是預算問題,不是飛不完。
  #      timeout    真逾時:飛不完。
  #    退出碼跟著分開,stot_repeat.sh 靠它把 pass 強制 0 並印歸因。
  #    ⚠️ 2(timeout)保持原值 —— 既有呼叫端與歷史紀錄都認這個碼。
  if [ "$node_died" -eq 1 ]; then
    note="node_died"; rc=3
  elif [ "$nh" -ge 6 ]; then
    note="settle_cut"; rc=4
  else
    note="timeout"; rc=2
  fi
  echo "$TRIAL,,,,$nh,0,$note" >> "$CSV"
  case "$note" in
    node_died)
      echo "  ✗ tot_node(PID $TOT_PID)在結案前就不在了 —— **不是逾時**:"
      echo "     預算還沒用完,節點就崩潰 / 被外部 kill / 走不到結案。已命中 $nh/6。"
      echo "     ⚠️ 平行跑兩趟或用 pkill -x tot_node 收尾時,後啟動的那趟會殺掉"
      echo "        前一趟(見上方 TOT_PID 的說明)。節點 log 尾巴:"
      tail -15 "$LOG" ;;
    settle_cut)
      echo "  ✗ 已命中 $nh/6,但摘要行還沒印,預算就用完了 —— **不是飛不完**:"
      echo "     卡在 impact_settle=$IMPACT_SETTLE s 的續錄窗(寬限期也沒等到)。"
      echo "     修法:放大 WAIT_TICKS(現 ${WAIT_TICKS:-120} tick × 12 s)"
      echo "     或縮小 IMPACT_SETTLE。這一趟的飛行結果是有效的,只是沒收到摘要。" ;;
    *)
      echo "  ✗ 逾時(只命中 $nh/6);最後階段:"
      grep -E "★|就位對準環|交接終端" "$LOG" | tail -8 ;;
  esac
  # ⚠️ make_artifacts.sh 由 EXIT trap 負責,失敗路徑一樣會產(素材優先於歸因)。
  exit "$rc"
fi
MISS_MAX="$MISS_MAX" ALT_MAX="$ALT_MAX" SPREAD_MAX="$SPREAD_MAX" \
  TRIAL="$TRIAL" CSV="$CSV" TOT_LOG="$LOG" python3 "$SR/_kpi_parse.py"
# ⚠️⚠️ **航次一結束就立刻產進度素材。** 本專案的資料是覆寫式的:
#    /tmp/trial_launch.log 只留最後一趟、dataflash 在 SITL 不重開時多趟共用
#    同一份 .BIN、cmdlog 舊版路徑寫死。視覺產物不當場產生,下一趟一跑就沒了。
#    (P91 的資料是靠開跑前手動備份才留下來的。)
# ⚠️ 走 _artifacts_once 而不是直接呼叫:EXIT trap 用同一個一次性旗標,
#    正常路徑在這裡產完之後,trap 就會 no-op(順序維持原樣:素材在計分報表之前)。
_artifacts_once
echo "── 遙測基準 ──"; tail -1 "$CSV"
echo "── 真值基準 ──"; grep "真值基準" "$LOG" | tail -1
echo "── 觸地基準(驗收用這個)──"
# ⚠️⚠️ 這裡以前是 `| tail -12` —— 一個**固定行數預算**,而它早就沒有餘裕了:
#    degraded 區塊固定 8 行 + 每架 MISS/ESCAPE/UNKNOWN 一則警告 + 判定 1 行。
#    2 架 ESCAPE(SW1 形狀)= 剛好 12 行、零餘裕;3 架 ESCAPE = 13 行,tail -12
#    砍掉的第一行正好是 ①「觸地地速 平均(只算 N 架 HIT)」,收尾畫面於是從
#    「⚠️ 全員值 …」開頭 —— 正是被脫靶機墊高、本輪特地要降級的那個灌水數字,
#    而降級後的真值被砍掉。再多一則警告(UNKNOWN / partial-agl / 命中沒鎖存 /
#    mode 詞彙脫節)都會踩到同一條線。
#    把 12 改成 20 只是把線往後挪,下次加行照樣踩到 ⇒ 改成**哨兵擷取**:
#    stot_score 在收尾摘要之前印一行 ===STOT_SUMMARY_BEGIN===,這裡從它的下一行
#    取到結尾,行數自動伸縮。
# ⚠️ 哨兵不見(舊版 stot_score / 部署版沒同步)時**絕不可以靜默印空白** ——
#    那才是最糟的結果:航次結束時什麼都沒有,看起來像跑完了。退回印全文並出聲。
SCORE_TXT="${CSV_BASE}_score.txt"
python3 "$SR/stot_score.py" "${CSV_BASE}.csv" --log "$LOG" > "$SCORE_TXT"
if grep -q '^===STOT_SUMMARY_BEGIN===$' "$SCORE_TXT"; then
  sed -n '/^===STOT_SUMMARY_BEGIN===$/,$p' "$SCORE_TXT" | tail -n +2
else
  echo "⚠️⚠️ stot_score 收尾哨兵 ===STOT_SUMMARY_BEGIN=== 不見了(版本不符?"
  echo "     部署版 $SR/stot_score.py 沒跟 repo 同步?)—— 改印全文,不吞掉"
  cat "$SCORE_TXT"
fi
echo "   (完整計分報表:$SCORE_TXT)"
