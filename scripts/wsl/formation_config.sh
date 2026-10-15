# shellcheck shell=bash
# formation_config.sh — 編隊層生效值的**單一來源**。source 進來,不要執行。
#
# ═══════════════════════════════════════════════════════════════════════════
#  為什麼要有這個檔
# ═══════════════════════════════════════════════════════════════════════════
# 2026-08-24 實測:整套正確的編隊組態一直存在於 `stot_trial.sh`(批次路徑),
# 而 `rep_interactive.sh`(互動路徑)**一項都沒設** ⇒ 吃到 `swarm_ros2_up.sh`
# 的裸預設,而那些是**泡棉機時代**的值:
#
#     NOMINAL_SPEED=14   WAIT_SPEED=10   CATCH_SPEED=20      (m/s EAS)
#
# 而飛控的 AIRSPEED_MIN 是 **25** —— ArduPlane 的 DO_CHANGE_SPEED 只在
# AIRSPEED_MIN <= 目標 <= AIRSPEED_MAX 時才接受,區間外回 MAV_RESULT=4、整發丟掉。
# ⇒ **互動路徑上編隊層送出的每一發速度指令都被拒收**,而且完全靜默
#   (直連送出只知道「送出去了」,收不收要靠 COMMAND_ACK 側錄)。
#   這就是「編隊成形從未被證明」的完整解答。
#
# ⚠️⚠️ 節點自己的預設(backup_node.py:83-85)與 launch 的預設
#   (sar_failover_nodes.launch.py:119-121)**都是對的**:135/90/157。
#   只有 shell 把它們蓋掉。⇒ 這個檔的工作是讓兩條路徑吃到同一份值,
#   而不是再抄第四份。
#
# ⚠️ 一律用 ${VAR:-...} ⇒ 呼叫端先設好的值優先,這個檔只補沒設的。
#
# 用法:
#   N=2 . "$SR/formation_config.sh"        # 之後 export 那些變數給下游
#   V_SPACING=2500 N=2 . "$SR/formation_config.sh"

# ── 速度 ────────────────────────────────────────────────────────────────
# ⚠️⚠️ **速度指令是空速(EAS),幾何是地速,兩者在 3100 m 差 1.167 倍。**
#    DO_CHANGE_SPEED / nominal_speed / catch_speed 吃的都是 EAS,
#    而間距、追趕距離全是地面距離。混用會系統性差 17%。
#    sigma(3100 m) = 0.7345 ⇒ EAS = 地速 x sqrt(sigma) = 地速 x 0.857。
#    使用者 2026-07-29 定調:157 m/s 是**地速**。
#
# ⭐⭐⭐ 2026-08-28:三個檔位**隨機體剖面 AIRFRAME 走**。
#   stockjet 原廠 plane-jet(主線)。實測(節流閥釘住、60 s 穩態、sd 0.013):
#            平飛頂速 EAS 105.5(與高度無關)、可持續平飛下限 EAS 25.6、
#            T/W 1.4286;同條件地速 182m 107.1 / 1032m 116.4 / 3155m 143.1。
#   lowdrag  低阻力改裝,= 2026-08-27 之前的全部航次(157/135/157/90 原封不動)。
#
# ⚠️⚠️ stockjet **不能用 x0.857 這條換算**。0.857 = sqrt(sigma(3100)) 是標準
#    大氣的 EAS/地速比,而實測比值是 105.5/143.1 = **0.737**(差的 1.16 倍
#    就是皮托管少掉的 E2T 修正,見 swarm_sitl_up.sh 的 TECS_SYNAIRSPEED 段)。
#    ⇒ stockjet 的巡航空速是**宣告值 80**(使用者定調),地速 107 是它的實測
#      對應值,兩者不再由一條換算式綁在一起。硬套 0.857 會得到 92 而不是 80。
# ⚠️⚠️ stockjet 的追趕/等待也**不用 1.16 / 0.67 這兩個比例**去推:那兩個比例
#    是從 lowdrag「加速側 +21 / 減速側 -45」的不對稱權限算出來的,而那個不
#    對稱來自 T/W 0.274。stockjet 的 T/W 是 1.4286,權限分佈完全不同。
#    ⇒ 直接釘 95 / 55,與 wingman_node / master_node / backup_node /
#      sar_failover_nodes.launch.py 的 declare_parameter 預設**逐格相同** ——
#      「哪一條路徑餵的」不可以改變答案(two-launcher-paths-differ)。
#      95:≤ AIRSPEED_MAX 110(拒收線)、≤ 實測天花板 105.5(可達線)、> 巡航 80
#      55:0.69x 巡航,遠高於實測下限 25.6 與 AIRSPEED_MIN 25;
#          閉合率 95-55 = 40 = 0.50x 巡航(舊組態 67/135 = 0.496,同一量級)
case "${AIRFRAME:-stockjet}" in
  lowdrag)
    FORM_V_GS="${FORM_V_GS:-${SHOW_CRUISE_GS:-157}}"
    FORM_V="${FORM_V:-$(awk -v g="$FORM_V_GS" 'BEGIN{printf "%.0f", g*0.857}')}"
    FORM_CATCH="${FORM_CATCH:-$(awk -v v="$FORM_V" 'BEGIN{printf "%.0f", v*1.16}')}"
    # ⚠️⚠️ 等待速度**不可以**取巡航的對稱下緣。速度權限是不對稱的:
    #    加速靠剩餘推力(135 EAS 只有 +1.33 m/s^2),減速靠阻力(-2.62,快 2 倍),
    #    可用區間也不對稱 —— 加速側到 156 EAS(+21),減速側到 ~90 EAS(**-45**)。
    #    取 0.67 倍(90 EAS)⇒ 閉合率 157-90 = **67 m/s,是 0.84 倍那版的 3 倍**。
    #    90 EAS 遠高於失速(AIRSPEED_MIN 25),剩餘推力 61 N ⇒ 爬升餘裕充足。
    FORM_WAIT="${FORM_WAIT:-$(awk -v v="$FORM_V" 'BEGIN{printf "%.0f", v*0.67}')}"
    #: 實測可達地速天花板(lowdrag,THR_MAX=100 的那一批)
    FORM_GS_CEILING_DEF=193
    ;;
  *)
    FORM_V_GS="${FORM_V_GS:-${SHOW_CRUISE_GS:-107}}"
    FORM_V="${FORM_V:-80}"
    FORM_CATCH="${FORM_CATCH:-95}"
    FORM_WAIT="${FORM_WAIT:-55}"
    #: 實測平飛頂速 EAS 105.5 在 3155 m 量到的地速
    FORM_GS_CEILING_DEF=143
    ;;
esac

# 可容忍落後量 L_max ~= Delta_地速 * T_recover(T=30 s)。本機體是**加速度受限**
# 不是速度受限(速度差衰減時間常數 ~19 s、初始加速度僅 1.2 m/s^2)⇒ 再打 0.7 折。
FORM_LMAX="$(awk -v g="$FORM_V_GS" 'BEGIN{printf "%.0f", g*0.16*30*0.7}')"

# ── 幾何 ────────────────────────────────────────────────────────────────
# ⚠️⚠️ **間距與控制門檻要分開推。**
#      幾何(任務決定):V_SPACING、V_ANGLE、V_TAIL
#      門檻(速度權限決定):CATCH_RADIUS、CLOSE_TIME
#      到位判準(雜訊量級決定):FORMED_RADIUS
#    綁在一起的話,把間距拉到 4 km 會連帶把 catch_radius 拉到 4 km ⇒
#    速度斜坡的比例區被撐爆,僚機整個收斂過程都是全速追、沒有比例控制。
V_SPACING="${V_SPACING:-4000}"
V_ANGLE="${V_ANGLE:-35}"

# ⭐⭐ V 尾(備長 slot)的距離**與機數有關**,不是固定 1.5 倍。
#
#   n>=3:有真僚機,rank2 已在 ~8 km 外 ⇒ 備長排在它們之後,取 1.5x 間距。
#   n==2:**沒有僚機** —— `_roles(2)` 回 (master=1, wingmen=[], backup=2),
#         `wingman_node` 整個不被實例化。這時備長就是唯一的編隊夥伴,
#         它的距離**就是**編隊間距。取 1.5x 會讓實際間距比要求的多 50%。
#   ⚠️ 這一條是 n=2 之下唯一決定編隊間距的旋鈕 —— v_spacing_m 是
#      wingman_node 的參數,n=2 完全讀不到它。
if [ "${N:-6}" -le 2 ]; then
  V_TAIL="${V_TAIL:-$V_SPACING}"
else
  V_TAIL="${V_TAIL:-$(awk -v s="$V_SPACING" 'BEGIN{printf "%.0f", s*1.5}')}"
fi

# ⭐⭐ 雁型的側後方偏置(備長專用)。
# ⚠️ 預設 0 = 正後方 = **與 2026-08-24 之前逐位元相同的行為**。
#    n=2 要雁型必須明確設值 —— 側偏會帶坡度、過載 n=1/cos(phi)、誘導阻力
#    隨 n^2 上升,實測讓側偏最大的機爬升率掉 30%,所以不能無條件開。
V_TAIL_ANGLE="${V_TAIL_ANGLE:-0}"
V_TAIL_SIDE="${V_TAIL_SIDE:-1}"

# ⭐⭐⭐ 編隊高度的參考框（使用者 2026-08-25 定調）。
#
#   absolute  僚機被送到 formation_alt（3100）的絕對高度 —— 舊行為
#   relative  僚機被送到「長機當下高度 + V_TAIL_DZ」
#
# ⚠️⚠️ 舊行為下 backup 與 master **各自**對著同一個絕對常數爬，
#    誰也沒有參照誰 —— 那不是相對編隊。僚機從第一拍就被命令飛到
#    3100 m 而它在 0 m，於是整段全力爬升，同時還要橫向追 slot。
#    FORMB1/C1/D1 實測：僚機毛爬升 8132 m（比長機 5797 還多），
#    保留率卻只有 5-23%，而長機 47-60%。
#
# ⇒ 相對框之下目標永遠**可達**，而且側向展開的閘門也可以改看相對量。
# ⚠️ 預設 absolute = 逐位元舊行為。
ALT_MODE="${ALT_MODE:-absolute}"
V_TAIL_DZ="${V_TAIL_DZ:-0}"

# catch_radius = 速度斜坡的比例區寬度,由**閉合迴路的時間常數**決定,與間距無關:
#   tau_loop = catch_r / Delta。L_max(~528)給 tau ~ 24 s。
FORM_CATCH_R="${FORM_CATCH_R:-$FORM_LMAX}"
# formed_radius = 「已進入槽位」的判準,由**動態雜訊量級**決定(GPS + WP 滯後
# + 陣風),不是由間距等比例縮放。
# ⚠️ 設太小(如 20 m)會讓 formed 永遠不成立,長機一路停在 wait_speed;
#    設太大(> 間距)則沒有鑑別力 —— 容差球會把長機本身包進去。
FORM_FORMED_R="${FORM_FORMED_R:-200}"

# ── 巡邏幾何：長機飛的那個框 ────────────────────────────────────────
#
# ⭐⭐⭐ **轉彎半徑是實际上限死對間距的那一項。**
#
# slot 繞著長機航向轉，所以長機一轉彎，外側僚機就必須飛更長的弧：
#     v_outer = V * (R + S) / R
# 實測地速天花板約 193 m/s（THR_MAX=100），巡航 157 ⇒ 餘裕只有 36：
#     R_min = S * V / (V_ceiling - V) = 2500 * 157 / 36 = 10903 m
# ⇒ **轉彎半徑至少 10.9 km**，轉 90° 要飛 17.1 km / 109 s。
#
# ⚠️⚠️ 而 `turn_radius` 的裸預設是 **800 m** —— 差十四倍。
#    800 m 的轉角讓 slot 掃掠率 v/R = 0.196 rad/s，外側僚機需要
#    額外 490 m/s —— 物理上不可能。僚機每一個轉角都會被甩開。
#
# ⚠️ FORM1（2026-08-24）實測：長機航向在 600 s 內掃過
#    52°→47°→64°→10°→307°→224°→127°→14°→257°→187°，
#    而 slot 在 2500 m 的力臂上—— 航向轉 90° 讓 slot **瞬移 3500 m**。
#    僚機在追一個會瞬移公里級的目標，永遠收斂不了。
#    （那一趢的直接原因是 `PATROL=0` ⇒ 沒有任何人在飛長機；
#      但就算有人飛，800 m 的轉角一樣會甩開僚機。）
#: 實測可達地速天花板。⚠️ 不是 AIRSPEED_MAX —— 那只是參數上限,真正的限制
#: 是推力/阻力。見 [[terminal-204-is-groundspeed]]。
#: ⚠️ 2026-08-28:預設隨剖面走(lowdrag 193 / stockjet 143,見上面那個 case)。
#:    193 是 lowdrag 量的,套到 stockjet 上會讓 TURN_RADIUS 被算得太小 ——
#:    而這一項的用途正是「外側僚機追不追得上」,高估天花板 = 高估追趕權限。
GS_CEILING="${GS_CEILING:-${FORM_GS_CEILING_DEF:-193}}"
# ⚠️ 推導**就用**天花板,不要另外打折:
#    我第一版用 0.9 倍「留餘裕」,半徑就從 10.9 km 跳到 23.5 km、
#    方框從 33 km 變成 70 km(繞一圈 280 km / 30 分鐘)—— 不切實際。
#    真正的問題是下面那道警告用 `need > 193` 去比一個**由 193 推出來的值**,
#    浮點上恰好相等就開火。⇒ 修比較方式,不是把推導改壞。
TURN_RADIUS="${TURN_RADIUS:-$(awk -v s="$V_SPACING" -v v="$FORM_V_GS" -v c="$GS_CEILING" 'BEGIN{m=c-v; if(m<5) m=5; printf "%.0f", s*v/m}')}"
# 方框邊長要容得下兩個 fillet 弧加上一段直線（給 slot 誤差收斂）。
# 取 3×R：兩個轉角各 R，中間至少一個 R 的直線段。
PATROL_BOX="${PATROL_BOX:-$(awk -v r="$TURN_RADIUS" 'BEGIN{printf "%.0f", r*3}')}"
# 入彎前多遠開始減速。⚠️ 裸預設 250 m 是配 800 m 半徑的；
#    半徑拉到 10.9 km 之後這個值相對地變得極小，跟著按比例放。
PRESLOW_MARGIN="${PRESLOW_MARGIN:-$(awk -v r="$TURN_RADIUS" 'BEGIN{printf "%.0f", r*0.3}')}"

# ⭐⭐⭐ 僚機的 DO_REPOSITION 盤旋半徑（2026-08-25）。
#
# ⚠️⚠️ 裸預設 30 m 在 157 m/s 下需要 atan(V^2/(g*R)) = **89.3°** 坡度
#    —— 物理上不存在。飛機只能頂在 ROLL_LIMIT_DEG=80 追一個永遠捕獲
#    不到的圈。FORMB1/FORMC1 實測（由 lat/lon 反解，不依賴 hdg 欄）：
#        角色   坡度p50   坡度p90   >60°的拍   高度保留率
#        長機    0.6°     12-14°    0.0%       52-53%
#        僚機   51.7°     74-75°   41-43%      13-23%
#    在 74° 坡度下 TECS_PITCH_MAX=15° 只換得到約 4° 航跡角。
#
# ⇒ 症狀是「留不住高度」不是「爬不動」：僚機毛爬升 8132 m
#   （比長機的 5797 還多），保留率卻只有 22.6% vs 長機 53.0%。
#
# 半徑由**可持續坡度**反推：R = V^2 / (g * tan(phi))
#   30° -> 4353 m ； 45° -> 2513 ； 60° -> 1451
# 取 30° 留足餘裕 —— 坡度越小，給 TECS 換航跡角的餘地越大
#（航跡角上限大致 ∝ pitch * cos(phi)）。
# ⚠️⚠️⚠️ **這個推導已經被實測推翻（2026-08-25, FORMD1）。**
#
#   假說：坡度太大 ⇒ TECS 換不到航跡角 ⇒ 留不住高度。
#   實驗：把半徑從 30 改成 4354（= 30° 可持續坡度）。
#
#              半徑 30      半徑 4354
#   坡度 p50    51.7°        41.5°    ↓ 機制成立
#   >60% 的拍   42%          26%      ↓
#   **保留率**   22.6%        5.1%    ⬇ **反而崩了**
#   **淨爬升**   3.04         0.27    ⬇
#   **d_slot**   639 m       5436 m   ⬇ 站位差 8 倍
#
#   ⇒ 坡度確實降了（半徑→坡度這一步成立），但
#     **坡度 → 保留率這一步被推翻**。大半徑的代價（繞著 slot
#     轉 4.4 km）遠大於坡度的好處。
#
# ⇒ 預設收回 30 = **逐位元舊行為**。推導的程式碼留著，
#   但要用它必須明設 `REPOSITION_BANK_DEG` —— 不可以默默套用
#   一個已經被實測否掉的值。
REPOSITION_BANK_DEG="${REPOSITION_BANK_DEG:-0}"
# 0 = 不推導，用裸預設 30（舊行為）。非 0 才由坡度反推。
if [ "${REPOSITION_BANK_DEG:-0}" != "0" ]; then
  REPOSITION_RADIUS_M="${REPOSITION_RADIUS_M:-$(awk -v v="$FORM_V_GS" -v b="$REPOSITION_BANK_DEG" 'BEGIN{r=b*3.14159265/180; printf "%.0f", v*v/(9.80665*sin(r)/cos(r))}')}"
else
  REPOSITION_RADIUS_M="${REPOSITION_RADIUS_M:-30}"
fi

# ── 高度與相位 ──────────────────────────────────────────────────────────
FORM_ALT="${FORM_ALT:-${SHOW_CRUISE_ALT:-3100}}"
PATROL_ALT="${PATROL_ALT:-$FORM_ALT}"
LAYER_SEP="${LAYER_SEP:-40}"
# 爬升/巡航相位閘門(給側向展開與爬升率分配用)
CRUISE_ALT_THRESHOLD="${CRUISE_ALT_THRESHOLD:-3000.0}"
CRUISE_GATE_HYST="${CRUISE_GATE_HYST:-150.0}"
CRUISE_GATE_DWELL="${CRUISE_GATE_DWELL:-5.0}"
EXPAND_TIME_S="${EXPAND_TIME_S:-120.0}"
CLOSE_TIME="${CLOSE_TIME:-60.0}"

FORMATION_VARS="FORM_V_GS FORM_V FORM_CATCH FORM_WAIT FORM_LMAX \
V_SPACING V_ANGLE V_TAIL V_TAIL_ANGLE V_TAIL_SIDE \
FORM_CATCH_R FORM_FORMED_R FORM_ALT PATROL_ALT LAYER_SEP \
CRUISE_ALT_THRESHOLD CRUISE_GATE_HYST CRUISE_GATE_DWELL EXPAND_TIME_S CLOSE_TIME \
TURN_RADIUS PATROL_BOX PRESLOW_MARGIN GS_CEILING \
REPOSITION_BANK_DEG REPOSITION_RADIUS_M ALT_MODE V_TAIL_DZ
REPOSITION_CHANGE_MODE GUIDED_MIN_ALT"

# ⚠️⚠️⚠️ 呼叫端**必須**在 source 之後呼叫 `formation_verify`。
#
# 為什麼不能寫在這個檔的頂層：
#   bash 對 `VAR=x . ./file` 這種寫法，賦值是**暫時的** ——
#   `.` 回來之後就還原。實測（2026-08-24, bash 5.1）：
#
#     N=2 V_SPACING=2500 . ./formation_config.sh    -> V_SPACING=<空>  V_TAIL=2500
#     N=2; V_SPACING=2500; . ./formation_config.sh  -> V_SPACING=2500  V_TAIL=2500
#
#   前者留下一組**互相矛盾**的值（間距空、V尾 2500），而且完全靜默。
#
# ⇒ 自檢寫在頂層的話，它跑在 source **內部** —— 那時暫時變數還看得見，
#   所以它**永遠不會觸發**。我第一版就是這樣寫的，而它的恆綠正好會被
#   讀成「沒問題」。同型：[[instrument-that-cannot-fire]]。
formation_verify() {
  local miss="" v val
  for v in $FORMATION_VARS; do
    eval "val=\${$v:-}"
    [ -z "$val" ] && miss="$miss $v"
  done
  if [ -n "$miss" ]; then
    echo "⚠️⚠️ formation_config.sh：以下變數 source 完仍然是空的：$miss" >&2
    echo "    最常見的原因：用了 VAR=x . ./formation_config.sh 這種**前綴式**。" >&2
    echo "    bash 對 special builtin 的賦值前綴是暫時的，. 回來就還原，" >&2
    echo "    而由它推出的其他值（如 V_TAIL）會留下來 ⇒ 一組矛盾的值。" >&2
    echo "    改成：  VAR=x; . ./formation_config.sh   或   export VAR=x" >&2
    return 1
  fi
  return 0
}


formation_echo() {
  local tag="${1:-編隊}"
  formation_verify || return 1
  # ⚠️ 「地速 X = EAS Y」在 stockjet 底下**不是**一條換算式的兩端:
  #    80 是宣告的巡航空速,107 是它在 3100 m 的實測對應地速(比值 0.737,
  #    不是標準大氣的 0.857)。剖面一併印出來,免得下次有人拿比值去反推。
  echo "[$tag] 速度(AIRFRAME=${AIRFRAME:-stockjet}):地速 ${FORM_V_GS} / EAS ${FORM_V}" \
       "/ 追趕 ${FORM_CATCH} / 等待 ${FORM_WAIT}"
  echo "[$tag] 幾何:間距 ${V_SPACING} m、半角 ${V_ANGLE}°、V尾 ${V_TAIL} m" \
       "@ ${V_TAIL_ANGLE}°(側 ${V_TAIL_SIDE})"
  if [ "${V_TAIL_ANGLE:-0}" != "0" ]; then
    echo "[$tag]   ⇒ 備長側偏 $(awk -v d="$V_TAIL" -v a="$V_TAIL_ANGLE" \
        'BEGIN{printf "%.0f", d*sin(a*3.14159265/180)}') m、" \
      "落後 $(awk -v d="$V_TAIL" -v a="$V_TAIL_ANGLE" \
        'BEGIN{printf "%.0f", d*cos(a*3.14159265/180)}') m"
  else
    echo "[$tag]   ⚠️ V_TAIL_ANGLE=0 ⇒ **正後方縱列,不是雁型**"
  fi
  echo "[$tag] 門檻:catch_r ${FORM_CATCH_R}、formed_r ${FORM_FORMED_R}、close_time ${CLOSE_TIME}s"
  echo "[$tag] 巡邏:轉彎半徑 ${TURN_RADIUS} m、方框邊長 ${PATROL_BOX} m、入彎減速 ${PRESLOW_MARGIN} m"
  if [ "${REPOSITION_BANK_DEG:-0}" != "0" ]; then
    echo "[$tag] 僚機盤旋半徑 ${REPOSITION_RADIUS_M} m(由 ${REPOSITION_BANK_DEG}° 可持續坡度反推)"
    echo "[$tag]   ⚠️⚠️ 這個推導已被 FORMD1 實測推翻:坡度確實降了(42%→26% 的拍 >60°),"
    echo "[$tag]      但保留率 22.6%→5.1%、d_slot 639→5436 m。你正在重跑一個被否掉的組態。"
  else
    echo "[$tag] 僚機盤旋半徑 ${REPOSITION_RADIUS_M} m(裸預設)"
  fi
  if [ "$ALT_MODE" = "relative" ]; then
    echo "[$tag] 高度參考框:**相對長機** + ${V_TAIL_DZ} m"
  else
    echo "[$tag] 高度參考框:絕對 ${FORM_ALT} m(僚機與長機各自對著同一個常數爬)"
  fi
  # ⚠️ 轉彎半徑不足時 slot 掃掠率會超出僚機的速度餘裕 —— 每個轉角都甩開僚機。
  # ⚠️ 容差 1.001:`TURN_RADIUS` 本來就是由 `GS_CEILING` 反推的,
  #    浮點上 `need` 會**恰好等於**天花板 ⇒ 裸 `>` 會對自己算出來的值開火。
  #    一道會對正確輸入報警的守衛,下一個人只會把它刪掉。
  awk -v r="$TURN_RADIUS" -v s="$V_SPACING" -v v="$FORM_V_GS" \
      -v c="$GS_CEILING" -v t="$tag" 'BEGIN{
    need = v*(r+s)/r
    if (need > c*1.001)
      printf "[%s] ⚠️⚠️ 轉彎半徑 %s m 之下外側僚機需要 %.0f m/s(天花板 %s)⇒ 轉角必甩隊\n", t, r, need, c
  }'
  # ⚠️ 這一條要大聲:formed_r 若 >= 間距,容差球會把長機本身包進去 ⇒
  #    「到位」判準沒有鑑別力,隊形怎麼亂都會回報成形。
  awk -v f="$FORM_FORMED_R" -v s="$V_TAIL" -v t="$tag" 'BEGIN{
    if (f+0 >= s+0)
      printf "[%s] ⚠️⚠️ formed_r %s >= V尾 %s ⇒ 到位判準沒有鑑別力(容差球含長機)\n", t, f, s
  }'
}
