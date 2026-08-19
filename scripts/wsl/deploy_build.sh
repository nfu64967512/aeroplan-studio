#!/usr/bin/env bash
# deploy_build.sh — 把 repo 的節點檔部署到 WSL 並**帶驗證地**重建。
#
# ⚠️⚠️ 為什麼需要這支(這一輪踩過的教訓):
#   `colcon build --symlink-install` 對本 package 的佈局**不保證一致**。
#   實測:`build/sar_swarm/sar_swarm/tot_node.py` 是**一般檔案(副本)**,不是
#   symlink,而 ROS2 實際執行的就是 build/ 那一份(見 tot_node 的 traceback
#   路徑)。⇒ 改了 src/ 而 colcon 判定「不需重建」時,跑的是**舊碼**,
#   而且沒有任何訊息。這正是本輪反覆踩到的靜默失敗形態。
#
#   更糟的是「刪檔案」的情況:build/ 會留著已從 src/ 刪除的模組,
#   import 照樣成功 —— 舊行為悄悄復活。
#
# ⇒ 本腳本的保證:
#   1. repo → src/ 逐檔複製後**比對 md5**
#   2. 預設**整個刪掉** build/<pkg> 與 install/<pkg> 再建(CLEAN=0 可關)
#   3. 建完再比對 src/ 與 build/ 每一個 .py 的 md5
#   4. 任何一步對不上就**非零退出並印出差異**,不靜默降級
#
# 用法:
#   bash deploy_build.sh                  # 乾淨重建(建議,~30 s)
#   CLEAN=0 bash deploy_build.sh          # 增量(快,但只在你確定沒刪檔時用)
#   PKG=sar_swarm bash deploy_build.sh
set -o pipefail

REPO="${REPO:-/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/scripts/wsl}"
WS="${WS:-$HOME/fixwing_ws}"
PKG="${PKG:-sar_swarm}"
SRC="$WS/src/$PKG/$PKG"
LAUNCH_DST="$WS/src/$PKG/launch"
CLEAN="${CLEAN:-1}"

NODES="master_role.py master_node.py wingman_node.py backup_node.py
       geodesy.py phase_gate.py preflight_contracts.py mavros_interface.py
       tot_node.py strike_authority.py contract_table.py predictor.py"
LAUNCHES="sar_failover_nodes.launch.py"
# sitl_run 下的執行腳本(不進 ROS2 package,但同樣要保證一致)
# ⚠️⚠️ **本腳本必須把自己列進來。** 第一版沒列 ⇒ 我在 repo 加了
#    preflight_check.py 到清單裡,但 WSL 跑的是舊的 deploy_build.sh,
#    所以那個檔從來沒被複製過 —— 防靜默失敗的工具自己踩了靜默失敗。
#    (症狀:stot_trial.sh 呼叫 preflight_check.py 得到 "No such file",
#     而 deploy 卻回報「✅ 部署一致」,因為它只驗 ROS2 package 那一層。)
# ⚠️⚠️ 2026-08-06 再犯一次,而且更隱蔽:漏了 **_kpi_parse.py**。
#    它不是「不存在」而是**部署版停在 8/4**,只認舊哨兵
#    「俯衝命中紅心:全員命中」;而 tot_node 只在「零 miss + 全部低空觸地」
#    那一種結案形狀才寫這句 ⇒ **其餘三種形狀(高空掠過 / 部分 MISS / 全部
#    MISS)一律被記成 no_summary**,一趟 5/6 命中的航次在遙測基準上長得
#    跟 0 命中一樣。而 stot_trial.sh 呼叫它時 **rc 是丟掉的**,失敗還會讓
#    `tail -1 $CSV` 印出**上一趟**的成績。
#    另外漏的 swarm_ros2_down.sh / swarm_sitl_down.sh 是收尾用的。
#    ⇒ 清單是手維護的,所以會再漏。用 tests/test_deploy_manifest.py
#      自動比對「鏈路引用到的檔案」vs「清單」,不要再靠記得。
# ⚠️⚠️ 2026-08-07 第三次漏檔,而且這次是**測試本身在騙人**。
#    這一批的呼叫樣式是 `( cd "$SR" && … python3 jet_show_cruise.py )`
#    —— 前面先 cd 進去了,所以**沒有 $SR/ 前綴**。測試的第一版只比對
#    `$SR/<file>` 字面樣式 ⇒ 掃不到它們,卻照樣全綠。
#    實際後果:jet_show_cruise.py 的「N 寫死 6」當天修好卻從未部署,
#    WSL 一直跑舊版(jet_rescatter / stot_kpi_loop / stot_rescore_all 同樣)。
#    測試已改成兩種樣式都掃,且含跨行的 `\` 續行(實測:單行樣式只抓得到
#    jet_show_ros2.py,那兩支的 python3 在下一行)。
# ⚠️ 註解不可以寫進下面那個清單變數的雙引號字串**裡面** —— 註解裡的引號
#    會把字串切斷,bash -n 會報 unexpected EOF。
# ⚠️ 也不要在本檔任何地方寫出「清單變數名 + 等號 + 引號」的字面樣式:
#    tests/test_deploy_manifest.py 用正規式抓第一個出現的地方,寫在註解裡
#    會讓它抓到註解而不是真正的清單(踩過一次)。
SCRIPTS="deploy_build.sh run_west.sh repro_two1.sh rep_interactive.sh stot_trial.sh stot_repeat.sh swarm_sitl_up.sh swarm_ros2_up.sh
         jet_catapult_launch.py jet_show_ros2.py jet_design.py
         stot_stage2_report.py stot_covariate.py sep_envelope.py stot_score.py
         preflight_check.py contract_verify.py make_artifacts.sh
         _kpi_parse.py swarm_ros2_down.sh swarm_sitl_down.sh
         jet_rescatter.py jet_show_cruise.py
         stot_kpi_loop.sh stot_rescore_all.py _show_anim.py
         stot_mission_anim.py stot_extract_df.py
         _tot_plot.py _dive_plot.py _formation_topview.py
         _formation_from_show.py stot_stage2_report.py"

fail=0
note() { printf '  %s\n' "$*"; }

echo "══ 1) repo → src/ ══"
for f in $NODES; do
  s="$REPO/sar_swarm_patches/$f"
  [ -f "$s" ] || { note "⚠️ repo 沒有 $f —— 跳過"; continue; }
  cp "$s" "$SRC/$f"
  a=$(md5sum "$s" | cut -d' ' -f1); b=$(md5sum "$SRC/$f" | cut -d' ' -f1)
  [ "$a" = "$b" ] || { note "✗ 複製後 md5 不符:$f"; fail=1; }
done
for f in $LAUNCHES; do
  cp "$REPO/sar_swarm_patches/$f" "$LAUNCH_DST/$f" || fail=1
done
for f in $SCRIPTS; do
  if [ ! -f "$REPO/$f" ]; then note "⚠️ repo 沒有 $f"; fail=1; continue; fi
  cp "$REPO/$f" "$WS/sitl_run/$f"
  # ⚠️ 這一層先前只 cp 不驗證 —— 而 ROS2 package 那一層的 md5 驗證
  #    完全涵蓋不到 sitl_run/,所以漏檔會回報「部署一致」。
  a=$(md5sum "$REPO/$f" | cut -d' ' -f1); b=$(md5sum "$WS/sitl_run/$f" | cut -d' ' -f1)
  [ "$a" = "$b" ] || { note "✗ sitl_run 複製後 md5 不符:$f"; fail=1; }
done
note "節點 $(echo $NODES | wc -w) 檔、launch $(echo $LAUNCHES | wc -w) 檔、腳本 $(echo $SCRIPTS | wc -w) 檔"

echo "══ 2) 重建($([ "$CLEAN" = 1 ] && echo '乾淨:先刪 build/ 與 install/' || echo '增量')) ══"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WS" || exit 1
if [ "$CLEAN" = "1" ]; then
  rm -rf "build/$PKG" "install/$PKG"
fi
colcon build --packages-select "$PKG" --symlink-install 2>&1 | tail -3

echo "══ 3) 驗證 src/ ≡ build/ ══"
n_ok=0; n_bad=0
for f in "$SRC"/*.py; do
  b="$WS/build/$PKG/$PKG/$(basename "$f")"
  if [ ! -f "$b" ]; then note "✗ build/ 缺少 $(basename "$f")"; n_bad=$((n_bad+1)); continue; fi
  if [ "$(md5sum "$f" | cut -d' ' -f1)" = "$(md5sum "$b" | cut -d' ' -f1)" ]; then
    n_ok=$((n_ok+1))
  else
    note "✗ 不一致:$(basename "$f")  src=$(md5sum "$f" | cut -c1-8) build=$(md5sum "$b" | cut -c1-8)"
    n_bad=$((n_bad+1))
  fi
done
# ⚠️ 反向檢查:build/ 有而 src/ 沒有 = 已刪除的模組還活著,import 會成功
for b in "$WS/build/$PKG/$PKG"/*.py; do
  [ -f "$SRC/$(basename "$b")" ] || { note "⚠️ build/ 殘留已刪除的 $(basename "$b") —— 舊行為可能復活"; n_bad=$((n_bad+1)); }
done
note "一致 $n_ok 檔;不一致/殘留 $n_bad 檔"
[ "$n_bad" -eq 0 ] || fail=1

echo "══ 4) 冒煙測試(import + 語法)══"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
python3 - <<'PY' || fail=1
import importlib, sys
mods = ["sar_swarm.geodesy", "sar_swarm.phase_gate",
        "sar_swarm.preflight_contracts", "sar_swarm.mavros_interface",
        "sar_swarm.wingman_node", "sar_swarm.tot_node",
        # ⚠️ 契約表必須 import 得到,否則 contract_verify.py 會退回相對路徑
        #    去讀**未經 md5 驗證**的 repo 副本 —— 契約層自己吃到舊參數。
        "sar_swarm.contract_table",
        # ⚠️ 預測器是 Hybrid Trajectory Shaping 的地基。import 壞掉時
        #    節點會**靜默退回**沒有預測的舊路徑,所以要納入冒煙測試。
        "sar_swarm.predictor"]
bad = []
for m in mods:
    try:
        importlib.import_module(m)
    except Exception as e:
        bad.append(f"{m}: {e!r}")
print("  import OK" if not bad else "  ✗ import 失敗:\n    " + "\n    ".join(bad))
sys.exit(1 if bad else 0)
PY

# ══ 5) ⭐⭐ 節點**真的起得來**嗎(帶生產參數)══════════════════════════
#
# ⚠️⚠️ import OK **不代表節點起得來**。RRA(2026-08-16)整批 8 趟、約 5 小時
#    報廢,而 import 冒煙測試是綠的 —— 因為問題出在 **ROS2 參數覆寫**:
#      ① `-p release_sigma_spec:=`(空字串)⇒ rcl「Couldn't parse」⇒ 起不來
#      ② 改成哨兵 "off" ⇒ rcl 把它解析成**布林** ⇒ InvalidParameterTypeException
#    兩者的症狀都是「tot_node 沒有進入 ready_for_target」,
#    而那長得跟埠號被別的 client 搶走**一模一樣**(fanout-port-exclusivity)。
#
# ⇒ 真的把節點起起來,餵**與批次相同**的參數集,只看參數有沒有被接受。
#    ⚠️ timeout 送 SIGTERM 造成的 ExternalShutdownException 是**正常關閉**,
#       不可以拿它當失敗判準(我第一版就是這樣誤報的)。
if [ "$fail" -eq 0 ]; then
  echo "══ 5) 節點啟動冒煙(生產參數)══"
  _out=$(timeout 12 ros2 run sar_swarm tot_node --ros-args \
      -p n:=2 -p release_sigma_spec:="none" -p ring_pace:="False" \
      -p tgo_from_predictor:="False" -p itcg_on:="False" 2>&1 || true)
  if echo "$_out" | grep -q "InvalidParameterType\|Couldn't parse parameter"; then
    echo "  ✗✗ **參數被拒** —— 節點起不來,批次會全趟失敗:" >&2
    echo "$_out" | grep -E "InvalidParameterType|Couldn't parse" | head -3 >&2
    fail=1
  elif echo "$_out" | grep -q "tot_node up"; then
    echo "  節點起動 OK(參數全部被接受)"
  elif echo "$_out" | grep -q "ConnectionRefusedError\|Errno 111"; then
    # ⚠️⚠️ 這一步**不可以**要求 SITL 先起來。
    #    部署是在每趟 SITL 之前跑的(rep_interactive.sh:280 在起 SITL 之前),
    #    而 tot_node 建構到一半就會去連 MAVLink TCP ⇒ 沒有 SITL 必然 Errno 111。
    #    第一版寫成「沒印 up 就算失敗」,結果是:
    #      單獨手跑 deploy_build.sh → 剛好有殘留 SITL → 綠;
    #      走 rep_interactive.sh → SITL 還沒起 → 紅 ⇒ 整批中止。
    #    **綠或紅取決於上一趟有沒有清乾淨**,那不是閘門,是擲骰子。
    # ⇒ 這一步要驗的是**參數層**(RRA 那顆地雷:`-p x:=` 解析失敗 /
    #   哨兵字串被當 bool ⇒ InvalidParameterTypeException ⇒ 節點整個起不來)。
    #   參數在建構的最前面就處理完了,所以「連得到 MAVLink 之前沒有參數錯誤」
    #   已經足夠涵蓋它;能走到 TCP 連線就代表參數全過。
    echo "  節點起動 OK(參數全部被接受;SITL 未起,只驗到 MAVLink 連線之前)"
  else
    echo "  ✗ 節點既沒印 up、也沒走到 MAVLink 連線 —— 起動路徑有問題" >&2
    echo "$_out" | tail -5 >&2
    fail=1
  fi
fi

if [ "$fail" -eq 0 ]; then
  echo "══ ✅ 部署一致,可以開跑 ══"
else
  echo "══ ❌ 有不一致 —— **不要開跑**,先修上面的項目 ══" >&2
fi
exit "$fail"
