#!/usr/bin/env bash
# run_west.sh — 海面目標(桃園往西 130 km)的基準航次驅動。
#
# ⚠️⚠️ 這支先前只存在於臨時 shell,交接文件引用了它卻沒有人把它存進 repo,
#    下一輪接手時整組幾何參數就消失了(phase_report.py / coupling.py /
#    cmd_speed_check.py 等分析工具也是同一個原因不見的)。
#    現在的幾何是從 P91 的 trial log 逐項反推回來的:
#      「扇形 R=35000 ±25° 6 slot 間隔 6109m;pushover R=6084 perch=3100m
#        → 目標 (25.0777, 119.94315)」
#      「auto_ring:perch 3100m / 角度 27° → pushover_r 480 → 6084m」
#
# 為什麼是海面目標(已確立的設計決策,不要改回山區):
#   紅心地表 z_g ≈ −32 m,幾何乾淨可重現;山區起飛點的 z_g 是 −1886 ~ −2703,
#   而且換一個起飛點就變 —— 環半徑漏算地形落差曾讓殘餘高度恆為 1784 m。
#
# 用法:
#   PREFIX=X TRIALS=1 bash run_west.sh              # 單趟
#   PREFIX=R TRIALS=3 bash run_west.sh              # 建立統計信心(散布雜訊 ±10 s)
#   CRUISE_GATE_HYST=0 CRUISE_GATE_DWELL=0 ... bash run_west.sh   # 關掉遲滯做 A/B
#
# ⚠️ 一個變數只准出現一次(重複定義時後面的勝)。所有旗標都是 ${VAR:-預設},
#    可從命令列覆寫做對照實驗。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"

# ── 起飛點與目標 ──────────────────────────────────────────────────────
# LOC=TAOYUAN → 25.0777,121.2325,32,52。目標取同緯度、往西 130 km:
#   Δlon = 130000 / (111320 × cos 25.0777°) = 1.2892°
export LOC="${LOC:-TAOYUAN}"
export TGT_LAT="${TGT_LAT:-25.0777}"
export TGT_LON="${TGT_LON:-119.94315}"

# ── 階段 2:ROS2 協同編隊巡航 ────────────────────────────────────────
# ⚠️ CRUISE_MODE=ros2 才會起 master/wingman 節點。scatter/formation 兩個模式
#    都不跑協同層 —— 用錯模式會「看起來有跑」但階段 2 根本沒被測到。
export CRUISE_MODE="${CRUISE_MODE:-ros2}"
export SHAPE="${SHAPE:-V}"
# ⚠️ 兩個逾時都要設:CRUISE_TIMEOUT 是外層 `timeout` 的上限,
#    SHOW_TIMEOUT 是 jet_show_ros2.py 自己的迴圈上限(預設 900)。
#    只設外層的話實際窗口仍然是 900 s —— Q1 就是這樣少跑了 400 s。
export CRUISE_TIMEOUT="${CRUISE_TIMEOUT:-1300}"
export SHOW_TIMEOUT="${SHOW_TIMEOUT:-$CRUISE_TIMEOUT}"
export SHOW_CRUISE_ALT="${SHOW_CRUISE_ALT:-3100}"
# ⚠️⚠️ 垂直通道協同(長機爬著等)**預設是開的**,但受控實測早就推翻它:
#    抵扇形散布 119.0 s vs 關閉 14.1 s(見交接文件 §4)。
#    實測 U1(沒關):僚機一落後就觸發,長機一路爬到 **4601 m**
#    (= CRUISE_ALT 3100 + SHOW_VERT_DZ_MAX 1500),而僚機還在 2100 m ——
#    垂直落差 2.5 km。已推翻的東西不要靠「預設值剛好沒觸發」來避開。
export SHOW_VERT_COORD="${SHOW_VERT_COORD:-0}"
# ⚠️⚠️ **協同層接手之前必須先爬完巡航高度。** 這一項在交接文件的 §6 沒有列,
#    Q1 因此沒設 ⇒ jet_catapult_launch.py 只爬到 ~100 m 就交棒,
#    整趟階段 2 都在爬升中跑(UAV6 全程在 1276 m),末端誤差 44 km。
#    那個數字與導引邏輯無關,是上游沒過就往下量的典型誤診。
export CLIMB_TO_ALT="${CLIMB_TO_ALT:-$SHOW_CRUISE_ALT}"
# ⚠️⚠️ 爬升段的速度指令會**留在飛控上**:`jet_catapult_launch.py` 的爬升區塊
#    送一次 `DO_CHANGE_SPEED CLIMB_TO_V`(70 EAS)之後就不再管它。
#    實測 T1:六機整段巡航被釘在 84–95 m/s 地速(= ~72 EAS ≈ CLIMB_TO_V),
#    儘管 wingman_node 每 3 s 就送一次 157 EAS。
#    ⇒ 爬升**速度**維持最佳爬升值(不要動,動了階段 1 就退步),
#      改成爬完之後把速度**交棒回巡航值**(CLIMB_DONE_V,與 nominal 對齊)。
export CLIMB_DONE_V="${CLIMB_DONE_V:-135}"

# ── ⭐⭐ 高速改裝(這幾項先前**從來沒有生效過**)────────────────────
# `swarm_sitl_up.sh` 只在 JET_CDP/JET_THRMAX **非空**時才套用,而
# `stot_repeat.sh` 傳的是空字串 ⇒ 幾十趟跑的都是**原廠 plane-jet 的阻力**。
# 實測 G1 巡航時窗(1330 s):油門 p50 **91%**、飽和 40.6%、SEP≈0、
# EAS 頂在 **98.5** —— 機體是真的推不動,不是指令沒到(指令全程 157 EAS)。
# 157 m/s 地速 @3100 m = **EAS 135**,需要 (135/98.5)³ ≈ 2.6 倍功率。
#
# jet_design.py --cruise 135 --terminal 204 --angle 27 反推:
#   c_drag_p 0.01642 / THR_MAX 25 / AIRSPEED_MAX ≥ 224
# ⚠️ --cruise 要餵 **EAS**(工具用海平面密度算);餵 157 會設計成 3100 m 的 183。
export JET_CDP="${JET_CDP:-0.01642}"
export JET_THRMAX="${JET_THRMAX:-25}"

# ── 爬升(階段 1)──────────────────────────────────────────────────
# ⚠️⚠️ 火箭助推到這一刻才真的成立:基準 25(巡航設計值)⇒ 爬升 35 是**加推力**。
#    先前基準是 100,所謂「助推 35」其實是把推力**砍到 35%**,方向完全相反。
# ⚠️ 彈射瞬間要更多推力:THR_MAX 是**伺服硬上限**,TKOFF_THR_MAX 突破不了它。
#    實測記錄:THR_MAX=19 時彈出去只能一路彈地。完整序列 60 → 35 → 25。
export TKOFF_THR_BOOST="${TKOFF_THR_BOOST:-60}"
export CLIMB_THR_MAX="${CLIMB_THR_MAX:-35}"
# 爬完要收回巡航設計值,否則整段巡航帶著助推推力(先前一直沒收,因為沒設)
export CRUISE_THR_MAX="${CRUISE_THR_MAX:-25}"
# 空速上限要蓋過終端 204(設計要求 ≥224),也要蓋過 catch_speed 157
export AIRSPEED_MAX_CAP="${AIRSPEED_MAX_CAP:-230}"
# 迴轉半徑 ∝ V²:135 EAS / 60° 傾角 = 1073 m(舊值 800 是低速時代訂的)
export TURN_RADIUS="${TURN_RADIUS:-1100}"
# ── 高度閘門:**由爬升段推導**,不是各自寫死 ────────────────────────────
# ⚠️⚠️ 2026-08-06 修掉的死區:彈射端(jet_catapult_launch.py)宣告「全員抵巡航
#    高度」的條件是 min_alt >= CLIMB_TO_ALT − CLIMB_TO_TOL,而 CLIMB_TO_TOL
#    預設 200 ⇒ CLIMB_TO_ALT=3100 時 **2900 m 就算爬完**。
#    但編隊層的 CRUISE_ALT_THRESHOLD 先前寫死 3000 ⇒ 中間有 **100 m 死區**:
#    彈射端說「爬完了、交棒」,編隊層卻仍判定「還在爬升」而不做協同讓速。
#    兩個數字先前是各自獨立寫死的,沒有任何東西保證它們相容。
#
#    ⇒ 單一真相源:CLIMB_TO_ALT 是唯一的設計高度,其餘全部由它推導。
#       彈射端保證的下限 = CLIMB_TO_ALT − CLIMB_TO_TOL
#       編隊層的閘門     = 該下限再往下讓 CRUISE_GATE_MARGIN,確保「彈射端說到了
#                          ⇒ 編隊層一定也認」,不留死區。
# ⚠️ 三個都仍可從命令列覆寫(對照實驗用),只是預設不再互相打架。
export CLIMB_TO_TOL="${CLIMB_TO_TOL:-200.0}"
export CRUISE_GATE_MARGIN="${CRUISE_GATE_MARGIN:-50.0}"
export CRUISE_ALT_THRESHOLD="${CRUISE_ALT_THRESHOLD:-$(awk \
  -v a="$CLIMB_TO_ALT" -v t="$CLIMB_TO_TOL" -v m="$CRUISE_GATE_MARGIN" \
  'BEGIN{printf "%.1f", a - t - m}')}"
# 遲滯與駐留。⚠️ 沒有遲滯時 P91 的 UAV5 抖了 10 次(見 sar_swarm/phase_gate.py);
#    更早的無遲滯版本有機在 200 s 內翻 20 次 ⇒ 末端誤差 35.6 km。
export CRUISE_GATE_HYST="${CRUISE_GATE_HYST:-150.0}"
export CRUISE_GATE_DWELL="${CRUISE_GATE_DWELL:-5.0}"
# 開跑前的相容性斷言:死區 = 編隊閘門高於彈射端保證的下限。
_climb_floor="$(awk -v a="$CLIMB_TO_ALT" -v t="$CLIMB_TO_TOL" 'BEGIN{printf "%.1f", a-t}')"
if [ "$(awk -v c="$CRUISE_ALT_THRESHOLD" -v f="$_climb_floor" \
        'BEGIN{print (c>f)?1:0}')" = "1" ]; then
  echo "✗ 高度閘門死區:CRUISE_ALT_THRESHOLD=${CRUISE_ALT_THRESHOLD}" \
       "> 彈射端保證的下限 ${_climb_floor}" \
       "(= CLIMB_TO_ALT ${CLIMB_TO_ALT} − CLIMB_TO_TOL ${CLIMB_TO_TOL})。"
  echo "  彈射端會宣告『全員抵巡航高度』並交棒,而編隊層仍判定在爬升 ——" \
       "協同讓速不會啟動,且完全靜默。"
  echo "  請調低 CRUISE_ALT_THRESHOLD、調小 CLIMB_TO_TOL,或提高 CLIMB_TO_ALT。"
  exit 1
fi
export MASTER_ALT_GATE="${MASTER_ALT_GATE:-0.0}"
# CLIMB_GATE_HI=-1 = 關閉動態爬升率限幅(助推之後不需要,留作對照)
export CLIMB_GATE_HI="${CLIMB_GATE_HI:--1}"

# ── 終端幾何(階段 4,由 P91 log 反推)──────────────────────────────
# ⭐⭐ 實驗一的**唯一變數**:ALIGN 的抵達判定半徑。
# ⚠️ ALIGN 的 `_reposition()` 送 param3=0 ⇒ ArduPlane 退回 `WP_LOITER_RAD`,
#    而 `WP_LOITER_RAD = JET_CRUISE²/(g·tan60°) = 159²/16.99 = 1488`,
#    再乘 eas2tas²(3100 m)=1.361 ⇒ **實際繞圈半徑 2026 m**。
#    實測平台 G1 2031±11 / HS1 2035±14 / NEW1 2028–2069 m(各 n≈3.5 萬),
#    與預測 ratio = 1.00。而抵達門檻 `d < reach` 的 reach 只有 260
#    ⇒ `len(self.arr)==6` 幾何上永遠不成立,**沒有 timeout、沒有錯誤訊息**,
#    四趟(G1/HS1/NEW1/P1)全部靜靜卡死在 ALIGN。
# ⇒ 先用 2400(平台 2035 + 18% 餘裕)驗證「reach 是唯一閘門」這個假設。
# ⚠️ 這是**治標**:站位精度會等於 ~2 km,STOT 對齊品質不可能達標。
#    治本是實驗二(讓 ALIGN 送明確半徑,或改成過環/最近點鎖存判定)。
#    先做這個只是為了**解鎖下游三個階段**,它們自 HS1 以來完全沒有資料。
export REACH="${REACH:-2400.0}"
export R_FAN="${R_FAN:-35000.0}"
export HALF_ANGLE="${HALF_ANGLE:-25.0}"
export PERCH_ALT="${PERCH_ALT:-3100.0}"
export TALT="${TALT:-3100.0}"
# 俯角 27°:計分的地速是水平分量 V·cosγ,27° 給 215.3 m/s(全域最大)。
# 45° 的漸近上限只有 194.5 < 204,90° 另有四個硬理由。不要改。
# ⚠️ 90° 是先前「硬打 90 度」實驗的殘留值,而 90° 會讓 tan(γ) 奇異、
#    pushover_r 塌成 0(V901 實測印出 −4.2e18),開跑前驗證一定擋下。
# ⚠️⚠️ 2026-08-06 訂正:上面整段論證的是 27°,而預設卻寫 45.0 —— 那是修
#    「90° 殘留」時留下的,和自己的論證打架。改回 27.0。
#    這不只是美觀問題:**唯一終端達標的航次 TWO1 就是 27°**
#    (2/2 命中、觸地誤差 1.95 / 4.04 m、殘餘高度 0.72 / 2.59 m)。
export DIVE_ANGLE="${DIVE_ANGLE:-27.0}"

# ⚠️⚠️ GATE_R 必須和 DIVE_ANGLE 一起改,不能只改角度。
#    環半徑 = (perch_alt − dive_alt) / tan(俯角):
#        45° ⇒ 3100 m、27° ⇒ 6084 m(地形重算後 6147 m)
#    真正決定成敗的是**比值** gate_r/環半徑,不是絕對距 —— 跨 17 趟零重疊:
#        ≥0.566 ⇒ 殘餘 44–343 m(成功側)   ≤0.446 ⇒ 殘餘 506–2499 m(失敗側)
#    ⇒ 只把角度改成 27° 而不動 GATE_R,比值會從 1200/3100 = 0.387
#      掉到 1200/6084 = **0.197** —— 比原本更深入失敗側。
#    ⚠️ 本檔先前**完全沒有** GATE_R ⇒ 落到 stot_trial.sh:23 的預設 1200,
#      而 AeroPlan 地圖右鍵打擊(rclick_strike → stot_ros2_bridge)跑的就是
#      裸的本檔、只覆寫座標 ⇒ 產品路徑一直在失敗帶。
#    4800 是 TWO1 實際生效值(4800/6147 = 0.781,成功側)。
#
# 【2026-08-13 4800 → 5600】⚠️ 上面「0.781 = 成功側」的推論**不成立**:
#    RPK/RPL 十二架次全部跑在 gate/環 = 0.781、交接距離 4765–4787 m
#    (比值零變異),結果卻是 2/6。TWO1 與 RPK/RPL 是**同一組設計幾何**
#    (gate 4800 / perch 3100 ⇒ γ_nominal 32.9°),成敗天差地遠
#    ⇒ 比值與 γ_nominal 都沒有鑑別力,不要再拿它們解釋成敗。
#
#    真正的決定量是**交接時的實際高度**:實測 2286–3510 m(±600 m 散布,
#    名目 perch_alt 3100,五架超調最高 +410)。交接距離固定 ⇒ 需求俯角
#    跟著散開 25.6°–36.3°,而實測分界是
#        γ ≤ 29.1° 三架全中(殘餘 0.75/1.23/4.06 m)
#        γ ≥ 33.8° 四架全敗(殘餘 412/623/860/1513 m)
#
#    拉大 gate_r 不治本(散布還在),但把**整個 γ 分布往下移**:
#        4800 ⇒ γ 25.5°–36.2°   最壞端深入必敗帶
#        5600 ⇒ γ 22.2°–32.1°   必敗尾巴被切掉
#    上界 5780(= 環半徑 6084 × 0.95),再大 PN 會在放行瞬間接手而退化;
#    contract_table.py 的 `gate_r < 環半徑` 不變式會擋。
#    ⚠️ TWO1 的重現不受影響:repro_two1.sh 自己明確帶 GATE_R=4800。
#
# 【2026-08-13 當日回退 5600 → 4800】RPO 三趟實測反證:上面那段只看 γ,
# 漏了 gate 與**守環擺盪**的關係。hold_radius 同時改成 1300 之後,飛機距紅心的
# 水平距在 [6147 ± 1300] = 4847–7447 擺盪,gate 5600 落在帶內 ⇒ 放行時常已在
# gate 內側 ⇒ 當場交棒 ⇒ 交棒距從 ±11 m 炸成 ±1075 m,真落地 5/12 → 1/6。
# 正確邊界:gate_r < 環半徑 − hold_radius = 4847。詳見 swarm_ros2_up.sh 同名參數註解。
export GATE_R="${GATE_R:-4700.0}"

# ── 動態交棒(可選;預設關,零行為變化)────────────────────────────────
# 上面的 4800 是把比值手動對到 TWO1 的 0.781 —— 但環半徑一被地形重算它就會漂
# (TWO1 實測 6084 → 6147,比值 0.789 → 0.781),而且換俯角就整個失效。
# 動態版改由「PN 段被要求飛多陡」反推交棒距,式子裡沒有設計俯角。
#   GATE_THETA_MAX=35 GATE_PHI=0.80 bash run_west.sh   # 開
# ⚠️ 35° 的依據:三趟六架次實測 PN 只交得出 23.3–33.9°。40° 沒有依據。
# ⚠️ 開啟後 GATE_R 在交棒判定上不生效(兩者互斥,不做 min/max)。
export GATE_THETA_MAX="${GATE_THETA_MAX:-0.0}"
export GATE_PHI="${GATE_PHI:-0.80}"

# ── ⭐⭐ 實驗一(治本,門檻 3):關掉俯衝段的**重發自激** ──────────────
# ⚠️⚠️ tot_node.py:1553-1556 的「逃生門」讓鎖存航點形同虛設:
#        if (pos[i][2] - z_line > line_tol and now - wp_t > resend_dt):
#            key = f"{key}#{int(now)}"      # 換 key ⇒ 重發 DO_REPOSITION
#    判斷式用的 z_line 是**幾何 45° 線**,而送出的航點高度是 track_frac=0.8
#    放大後的 −830 m —— 兩者不同基準。飛機只要沒照 45° 掉,條件就**恆為真**。
#    ⇒ 每 resend_dt(2.0 s)重發一次 ⇒ ArduPlane 的 prev_WP 被重錨成
#      「現在位置＋現在高度」⇒ 高度斜坡每 2 s 從當前高度重新起算,
#      下降需求**永遠累積不起來**。這是自激,不是調參問題。
#
# 實測指紋(RD1 dataflash,六機):
#   TECS.hin 每 **2.04 s** 被抹回當下高度(= RESEND_DT 2.0),在 2915–3215 m
#   之間鋸齒跳動;重發 15–22 次/機;TECS.hdem 全程 3040–3077 m **從未被要求下降**。
#   後果:離環到 PN 交接那 1950 m(30–46 s)六機**淨爬升 +54~+175 m**,
#   交接時 h/d = **2.41–2.75**(45° 線要求 1.0),高出線 +1694~+2106 m。
#   PN 只剩 1200 m 水平吃 2900–3300 m ⇒ 殘餘 567–1437 m。
#
# ⇒ LINE_TOL 拉到 3000 讓 :1554 的條件恆為假 = **真正的鎖存**,不動程式碼。
# ⚠️ 不改 RESEND_DT:拉長週期只是把鋸齒變粗,還會落在濾波器「安定/不安定」
#    的中間地帶,結果不好判讀。
# ⚠️ 已排除的嫌疑(不要順手一起改):送出的航點高度是**對的**
#    (dataflash MAVC.Z = −760~−894 m、Res=0 已接受);PN 俯角權限也是好的
#    (DIVE_PITCH_MIN=−90 確有第二次寫入、ATT.DesPitch 觸底 −90.0、實得 Pitch −79°)。
export LINE_TOL="${LINE_TOL:-3000.0}"

# ── ⭐⭐ 俯衝權限:三個開關要一起開,少一個就只有 6.6° ────────────────
# 實測 DV1(uav3 dataflash PARM):PTCH_LIM_MIN_DEG 與 TECS_PITCH_MIN 全程
# **停在 −20、只寫入 1 次(開機)** ⇒ 指令 27° 但實際航跡角只有 6.6°。
# 原因:tot_node 放開低頭權限的條件是 `if self.dive_pitch_min < 0.0:`,
# 而 DIVE_PITCH_MIN 預設 0.0 ⇒ 整段跳過,權限從未下放。
# ⚠️ 韌體端沒有擋:`PTCH_LIM_MIN_DEG` 的 @Range 是 **-90 0**,允許到垂直。
export DIVE_PITCH_MIN="${DIVE_PITCH_MIN:--90.0}"
export DIVE_PITCH_MAX="${DIVE_PITCH_MAX:-45.0}"
# 幾何前饋:指令俯角不准比視線淺。記憶明載「單改 DIVE_ANGLE 無效,
# 真開關是 PN_LOS_FLOOR」—— 純 PN 的縱向通道根本不引用 dive_angle。
export PN_LOS_FLOOR="${PN_LOS_FLOOR:-1.0}"
# 陡俯角要允許負的法向過載(推頭),否則 PN 拉不下去
export PN_NZ_MIN="${PN_NZ_MIN:--1.0}"
# 下沉率上限也要放開,否則 TECS 會用抬頭去限制下沉
export DIVE_SINK_MAX="${DIVE_SINK_MAX:-200}"

# ── ⭐⭐ 終端俯衝:地速 ≥204 m/s 且油門全開 ────────────────────────
# 三個限制器**同時**存在,少解一個就達不到 204:
#
# 1) THR_MAX —— `_pn_cmd` 送的 thr 是 0..1,韌體算 forced_throttle = thr×100
#    再被 servos.cpp 夾到 [THR_MIN, THR_MAX] ⇒ **送 1.0 只等於 THR_MAX**。
#    巡航設計值 25 之下「全油門」= 25%。tot_node 先前完全沒斷言 THR_MAX
#    (已補 dive_thr_max)。實測 E11:2000–2500 m 帶 ThO_max 只有 22.6。
export DIVE_THR_MAX="${DIVE_THR_MAX:-100.0}"
# 2) dive_full_thr —— 鎖定 thr=1.0,不再用油門調時間。
#    ⚠️ 代價:同時性失去油門這個執行器,改由各機不同的 perch 高度承擔。
#    使用者已定調「地速優先」,散布退居其次。
export DIVE_FULL_THR="${DIVE_FULL_THR:-True}"
# 3) pn_vmax —— `v_req = max(45, min(pn_vmax, R/t_rem))`。135 之下 PN 永遠
#    不會**要求** 204,油門律 thr = trim + gain×(v_req − V) 也就不會開大。
#    取 210 留餘裕;AIRSPEED_MAX 已是 230,不會再被飛控端夾住。
export PN_VMAX="${PN_VMAX:-210.0}"

# ⚠️⚠️ 目標在**台灣海峽海面**(桃園機場向西 ~130 km),海面 = 0 m AMSL,是已知常數。
#    絕不能讓 tot_node 去問地形資料庫 —— SRTM 在開闊水域沒有資料,回外插值:
#    同一個紅心座標六架各報一個高度(實測散布 17 m,均值偏低 3-15 m)。
#    釘死 0.0 才能談「打在海拔 0」。陸地目標請改回 nan(=查資料庫)。
export TARGET_AMSL="${TARGET_AMSL:-0.0}"

# ⚠️⚠️ PERCH 爬升空速。預設 max(vmin*1.45, 65)=65 m/s 是**舊泡棉機體**校出來的,
#    噴射機在這個速度撐不住高度。EL1 實測:UAV3 早到 160 s,在 PERCH 以 65.2 m/s
#    「爬升」,高度反而從 2603 m 一路掉到 519 m,永遠達不到就緒閘門,整趟卡死逾時。
#    sep_envelope.py 在 3100 m 量到 SEP 零交越在 EAS 72-73 —— 指令 65 落在
#    可持續飛行包絡**之外**。設 100 留足裕度(仍遠低於 AIRSPEED_MAX_CAP 165)。
#    ⚠️ 早到的飛機被罰:抵達散布越大,先到者在 PERCH 待越久、掉得越多。
export PERCH_CLIMB_V="${PERCH_CLIMB_V:-100.0}"

# ⚠️⚠️ 用「轉彎」耗時間,不要用「慢飛」耗時間 —— 這是本輪最重要的一項。
#    vmin 是 ALIGN/PERCH 的等待速度,同時也是 **weave 的啟動門檻**:
#    tot_node 只在 v_req < vmin 時才走 S 型機動,否則走 straight 慢飛。
#    舊值 45 m/s 有兩個問題:
#      ① 低於可持續飛行包絡(sep_envelope.py 量到 3100 m 的 SEP 零交越在
#         EAS 72-73)。要求飛機用飛不動的速度耗時間,結果是往下掉 ——
#         EL1 的 UAV3 就是這樣從 2603 m 掉到 519 m,整趟卡死。
#      ② v_req 中位數是 70 m/s > 45,所以 weave 幾乎不啟動(RD1/EL1 只有
#         12-17% 的 ALIGN 在 weave)。而 P91(唯一 6/6 命中、誤差 3.0 m 的
#         一趟)weave 佔 68.5%、時間膨脹 3.87x。
#    改 95 之後,RD1/EL1 條件下 100% 的 straight 都會改走 weave。
export VMIN="${VMIN:-95.0}"

# ⚠️ 換向週期。舊值 22 s 是**泡棉機時代**的:泡棉 14 m/s 的轉彎率 40.1 °/s,
#    θ=53° 只要 2.6 s ≪ 半週期 11 s。本機體 157 m/s 只有 3.58 °/s,
#    把航向從 +θ 甩到 −θ 要 2θ/ω:θ=45° 需 25.1 s > 半週期 11 s
#    ⇒ **θ > 19.7° 根本轉不完就被要求反向**,達成的 cosθ 遠大於指令值。
#    週期要 ≥ 4× 換向時間 ⇒ 120 s。
#    ⚠️ 代價:橫向擺幅 A = V·sinθ·P/4,θ=45°/P=120/V=100 ⇒ 約 2.1 km
#      (P91 在 P=22 之下只有 ~757 m,實測最近接近 348m/垂直 82m)。
#      擺幅放大 2.8x,所以同時加了執行期接近監測(見 tot_node 的 _proximity_check)。
export WEAVE_PERIOD="${WEAVE_PERIOD:-120.0}"
# 俯衝指令空速要高於目標地速,TECS 才會持續壓機頭追速(這是俯衝的動力來源)
export DIVE_SPEED="${DIVE_SPEED:-210.0}"

export PREFIX="${PREFIX:-X}"
export TRIALS="${TRIALS:-1}"

echo "[west] 目標 (${TGT_LAT}, ${TGT_LON}) ← 起飛 ${LOC}"
echo "[west] 階段2 CRUISE_MODE=${CRUISE_MODE} 窗口 ${CRUISE_TIMEOUT}s"
echo "[west] 高度閘門 ${CRUISE_ALT_THRESHOLD} m,遲滯 ${CRUISE_GATE_HYST} m /" \
     "駐留 ${CRUISE_GATE_DWELL} s ⇒ 退出線 " \
     "$(awk -v t="$CRUISE_ALT_THRESHOLD" -v h="$CRUISE_GATE_HYST" \
        'BEGIN{printf "%.0f", t-h}') m"
echo "[west] 終端 perch ${PERCH_ALT} m / 俯角 ${DIVE_ANGLE}° / 扇形 ${R_FAN} m"

exec bash "$SR/stot_repeat.sh"
