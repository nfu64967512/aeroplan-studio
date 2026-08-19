#!/usr/bin/env bash
# stot_repeat.sh — 同一組態連跑 N 趟,量成功率。
#
# 每趟都完整重啟 SITL(命中即消耗,機體留在地上,不重啟第二趟湊不齊六機)。
# 參數不在這裡定義 —— 一律透過環境變數傳給 stot_trial.sh,避免兩份參數各自漂移。
# 判定用 stot_score.py 的**觸地基準**(軌跡穿過 AGL=0 的那一點),不是「最近取樣點」。
#
# ⚠️ 2026-07-28:判定加入「俯衝地速」,而且它是第一優先(使用者定調)。
#    量測基準 = 六機**平均觸地地速**(不是峰值,也不是最慢那架)。
#    因此總表換到 stot_repeat_v2.csv:v1 少了速度欄,而且 v1 前段的列是用
#    「取第一個地面穿越點」的舊計分器寫的,已知會誤判(T1 記成 2083 m、
#    B21 記成 2554 m,重算後其實是 0.39 m 和 2.71 m)。v1 原檔保留不動,
#    要看修正後的歷史值請跑 stot_rescore_all.py。
#
# 用法:
#   PREFIX=P TRIALS=5 PN=True GATE_R=1200.0 DIVE_ANGLE=10.0 TRACK_FRAC=0.8 \
#     DIVE_SPDW=0.0 DIVE_PACE=50.0 LINE_TOL=25.0 HDEM_TC=0.5 DIVE_LAYER=4.0 \
#     bash stot_repeat.sh
# SPEED_MIN 可調(預設 100):六機平均觸地地速的門檻。
#
# ⚠️ 不要 set -u:source ROS2 setup.bash 會踩到未設定的 AMENT_TRACE_SETUP_FILES。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
export SR              # 內嵌的計分區塊也要用同一個路徑,不要各自寫死一份
TRIALS="${TRIALS:-5}"
PREFIX="${PREFIX:-R}"
# ⚠️ 不要放 /tmp:WSL 重啟會清空(已經被清掉一整輪的飛行記錄)。
OUT_DIR="${OUT_DIR:-$HOME/fixwing_ws/runs}"
export OUT_DIR
mkdir -p "$OUT_DIR"
SUMMARY="${SUMMARY:-$OUT_DIR/stot_repeat_v2.csv}"
SPEED_MIN="${SPEED_MIN:-100.0}"
MISS_MAX_ACC="${MISS_MAX_ACC:-5.0}"   # 觸地誤差門檻;使用者 2026-07-28 放寬到 10
# ⚠️⚠️ 機數。2026-08-06 之前整條批次鏈都寫死 6(這裡不轉發、閘門 -ne 6、
#    stot_trial.sh 也不傳 -p n:=)。漸進測試(2 機 → 3 → 4 → 6)必須從這裡
#    一路傳到底,少一段就會安靜地退回 6。
#    ⚠️ 名字用 SWARM_COUNT 而不是 N:sar_failover_nodes.launch.py 的角色分配
#       讀的就是這個環境變數(launch 的 for 迴圈要在建 description 時就知道 N)。
SWARM_COUNT="${SWARM_COUNT:-${N:-6}}"
export SPEED_MIN MISS_MAX_ACC SWARM_COUNT

[ -f "$SUMMARY" ] || echo "trial,n_touch,gs_touch_mean,gs_touch_min,gs_touch_max,max_impact_miss_m,max_abs_alt_m,touch_spread_s,pass" > "$SUMMARY"

for n in $(seq 1 "$TRIALS"); do
  T="${PREFIX}${n}"
  # ⚠️ PREFIX 撞名會讓計分器讀到**上一輪同名的舊 CSV**,報出一個和本趟完全無關的
  #    成績(實測:起飛被誤判失敗而中止,卻報出舊機體的 58.7 m/s / 0.39 m)。
  #    開跑前先把同名檔案清掉,寧可「沒有資料」也不要「錯的資料」。
  rm -f "$OUT_DIR/tot_${T}.csv" "$OUT_DIR/tot_${T}_geom.csv" \
        "$OUT_DIR/trial_${T}.log" "$OUT_DIR/df_${T}.csv" \
        "$OUT_DIR/show_${T}.csv" 2>/dev/null
  echo "════════════════ REPEAT $T ════════════════"
  # ⚠️ 這裡的 pkill 是「開跑前清場」,和 stot_trial.sh 收尾那個不同 ——
  #    此時本趟還沒啟動 tot_node,殺的是上一趟的殘留,語意正確。
  #    ⚠️ 但**平行跑兩趟時仍會互相殺**。要平行跑就必須改成 PID/cgroup 隔離。
  pkill -x tot_node 2>/dev/null
  bash "$SR/swarm_sitl_down.sh" >/dev/null 2>&1
  sleep 5
  # SITL 要有個常駐的父行程才不會被 PDEATHSIG 收掉 —— 本腳本就是那個父行程。
  # ⚠️ 機體改裝的變數(JET_CDP / JET_THRMAX / JET_CRUISE)一定要往下傳。
  #    以前這一行把 AIRSPEED_MAX_CAP 寫死成 150 也不轉發任何 JET_* ——
  #    結果是「以為在跑高速機體,其實整批跑的是原廠 plane-jet」,而且**完全沒有
  #    錯誤訊息**,只會看到速度就是上不去。
  # ⚠️ LOC 也要可覆寫:表演的起飛點必須在**目標之外**(D_home > r_fan > pushover_r),
  #    否則蜂群得先飛離目標再折返。桃園預設點距紅心只有 7 km,而扇形在 11 km、
  #    環在 8.6 km —— 起飛點等於在環裡面,整條航路是反的。
  # ⚠️⚠️ 2026-08-06:N 一定要往下傳。以前這一行不轉發 N,swarm_sitl_up.sh 就用
  #    自己的預設 6 —— 你以為在跑兩機漸進測試,實際上起的是六架,而且**沒有任何
  #    錯誤訊息**(和 JET_CDP 空字串那個坑是同一種形狀:上游少傳一個變數,
  #    下游安靜地用預設值)。
  ( cd "$SR" && JET=1 LAUNCH=catapult LOC="${LOC:-TAOYUAN}" \
      N="$SWARM_COUNT" \
      AIRSPEED_MAX_CAP="${AIRSPEED_MAX_CAP:-165}" \
      JET_CDP="${JET_CDP:-}" JET_THRMAX="${JET_THRMAX:-}" \
      JET_CRUISE="${JET_CRUISE:-}" JET_TKOFF_THRMAX="${JET_TKOFF_THRMAX:-}" \
      DIVE_SINK_MAX="${DIVE_SINK_MAX:-}" DIVE_ROLL_MAX="${DIVE_ROLL_MAX:-}" \
      bash "$SR/swarm_sitl_up.sh" > "/tmp/repeat_sitl_${T}.log" 2>&1 ) &
  sleep 150
  # ⚠️ 閘門也要跟著 N。寫死 6 的話,兩機測試永遠會判「SITL 起不來」而跳過整趟。
  if [ "$(pgrep -x arduplane | wc -l)" -ne "$SWARM_COUNT" ]; then
    # ⚠️ 這一列以前只有 6 欄,而表頭有 9 欄 —— DictReader 讀到的 pass 是 None,
    #    「不是 1」所以結論碰巧沒錯,但任何按欄位取值的下游都會拿到錯位的資料。
    echo "$T,0,,,,,,,0" >> "$SUMMARY"; echo "  ✗ SITL 起不來"; continue
  fi
  TRIAL="$T" bash "$SR/stot_trial.sh"
  trc=$?
  # ⚠️⚠️ 2026-08-05 blocker D(同型):以前這裡**完全不看** stot_trial.sh 的退出碼,
  #    下一行就直接計分,而計分只讀 CSV。於是「航次自己宣告失敗」與「航次正常
  #    結案」在總表上無從分辨:一趟節點在最後幾秒被砍掉、CSV 卻剛好湊到六筆
  #    觸地的航次,總表照樣可以記 pass=1。
  #    rc 對照(見 stot_trial.sh 結尾):
  #      1 = 前置/彈射失敗   2 = 真逾時   3 = 節點提前死亡   4 = impact_settle 窄縫
  #    ⚠️ 仍然照常計分 —— 數字要留著當證據,只是 pass 強制 0(見 TRIAL_RC)。
  export TRIAL_RC="$trc"
  if [ "$trc" -ne 0 ]; then
    echo "  ⚠️⚠️ $T:stot_trial.sh 以 rc=$trc 結束" \
         "(1=前置/彈射失敗、2=逾時、3=節點提前死亡、4=impact_settle 窄縫)" \
         "⇒ 本趟 pass 強制 0;歸因見 stot_trial 的 note 欄與 $OUT_DIR/artifacts/$T/"
  fi
  python3 - "$T" "$SUMMARY" <<'PY'
import json, subprocess, sys, os
T, SUMMARY = sys.argv[1], sys.argv[2]
sr = os.environ.get("SR") or os.path.expanduser("~/fixwing_ws/sitl_run")
OUT = os.environ.get("OUT_DIR", os.path.expanduser("~/fixwing_ws/runs"))
try:
    out = subprocess.run(
        ["python3", os.path.join(sr, "stot_score.py"),
         os.path.join(OUT, f"tot_{T}.csv"),
         "--log", os.path.join(OUT, f"trial_{T}.log"),
         "--miss-max", os.environ.get("MISS_MAX_ACC", "5.0"),
         "--speed-min", os.environ.get("SPEED_MIN", "100.0"), "--json"],
        capture_output=True, text=True, timeout=120).stdout
    s = json.loads(out)
except Exception as e:
    open(SUMMARY, "a").write(f"{T},0,,,,,,,0\n")
    print(f"  ✗ 計分失敗:{e}")
    raise SystemExit
vmin_req = float(os.environ.get("SPEED_MIN", "100.0"))
miss_req = float(os.environ.get("MISS_MAX_ACC", "5.0"))
per = s["per_uav"]
touch = [v for v in per.values() if v.get("impact_miss_m") is not None]
n_t = len(touch)
mx = max((v["impact_miss_m"] for v in touch), default=None)
alt = max((abs(v["alt_agl_m"]) for v in per.values()
           if v.get("alt_agl_m") is not None), default=None)
ts = [v["impact_t_s"] for v in touch]
spread = (max(ts) - min(ts)) if len(ts) > 1 else 0.0
# ⭐ 第一優先:六機**平均**觸地地速。取平均而不是最慢那架,是使用者的取捨;
#    最慢/最快仍寫進總表,免得平均把落後那一架蓋掉。
gs = [v["impact_gs_ms"] for v in touch if v.get("impact_gs_ms") is not None]
gmean = (sum(gs) / len(gs)) if gs else None
# ⚠️⚠️ 首航次全員命中是**前提**,不是加分項(2026-08-04 補上,對齊
#    stot_score:648 的 first_pass_ok 與 stot_rescore_all.verdict())。
#    少了這一條,一趟「兩架脫靶飛到 200 km 外、幾小時後隨機重穿紅心」的航次
#    在觸地基準上看起來很正常,總表會記 pass=1 而 stot_score 判 ❌。
#
# ⚠️⚠️ 2026-08-05 blocker C:**缺 verdict 一律 fail-closed,而且現在就踩得到。**
#    舊寫法 `True if not _v else …` 的理由是「--no-window 產的舊 JSON 沒有
#    verdict」,但上面那條命令**從來不傳 --no-window** —— 這裡「沒有 verdict」
#    的唯一成因是計分器版本不符:本區塊呼叫的是**部署版** $SR/stot_score.py,
#    實測 ~/fixwing_ws/sitl_run/stot_score.py 只有 15,140 B、grep 不到
#    verdict/window/all_hit,卻仍然接受 --miss-max/--speed-min/--json 且正常回 0。
#    ⇒ first_pass_ok 恆 True ⇒ 總表的 pass 欄**完全沒有首航次閘門**,
#      而總表正是拿來排序、挑「最好的一趟」的依據 = 幽靈達標的第二條路。
#      它不需要任何奇怪的 mode 列序,只需要「部署版 != repo 版」這個已知常態。
#    未判定 ≠ 通過:量不出來就記不通過,並吼一句版本不符(要看得見,不是 log 尾巴)。
_v = s.get("verdict")
if isinstance(_v, dict):
    first_pass_ok = bool(_v.get("all_hit"))
else:
    first_pass_ok = False
    print("  ⚠️⚠️ %s 的 JSON 沒有 verdict 區塊 ⇒ 首航次判定**量不出來**。"
          % os.path.join(sr, "stot_score.py"))
    print("     本區塊從不傳 --no-window,所以這是**計分器版本不符**"
          "(部署版沒跟 repo 同步)—— 本趟一律記 pass=0(未判定 ≠ 通過)。")
    print("     修法:把 repo 版 scripts/wsl/stot_score.py 部署到 %s 再重跑,"
          "或事後用 stot_rescore_all.py 重算。" % sr)
# ⚠️ 航次自己宣告失敗時(stot_trial.sh rc != 0)pass 強制 0 —— 見下方 TRIAL_RC。
trial_rc = os.environ.get("TRIAL_RC", "0")
# ⚠️⚠️ 「全員觸地」的分母必須是**本趟的機數**,不是寫死的 6。
#    寫死 6 的後果不是誤判通過,而是**兩機測試永遠 pass=0** —— 就算兩架都
#    完美命中,n_t 也只會是 2,條件恆假。看起來像「怎麼調都過不了」。
_want_n = int(os.environ.get("SWARM_COUNT", "6"))
ok = int(n_t == _want_n and mx is not None and mx <= miss_req and spread <= 3.0
         and alt is not None and alt <= 30.0
         and gmean is not None and gmean >= vmin_req
         and first_pass_ok
         and trial_rc == "0")
f = lambda x, p="%.2f": "" if x is None else p % x
open(SUMMARY, "a").write(
    "%s,%d,%s,%s,%s,%s,%s,%.2f,%d\n"
    % (T, n_t, f(gmean, "%.1f"), f(min(gs) if gs else None, "%.1f"),
       f(max(gs) if gs else None, "%.1f"), f(mx), f(alt), spread, ok))
print("  ▶ %s 觸地 %d/%d  地速平均 %s(最慢 %s)  誤差 max %s m  "
      "最低AGL max %s m  散布 %.2f s  → %s"
      % (T, n_t, _want_n, f(gmean, "%.1f"), f(min(gs) if gs else None, "%.1f"),
         f(mx), f(alt), spread, "✅" if ok else "❌"))
if not ok:
    # ⚠️⚠️ 這份清單先前只有三個分支(機數 / 首航次 / rc),而 `ok` 有**七個**條件
    #    ⇒ 真正沒過的若是誤差/散布/高度/地速,清單就是空的或指向錯的項。
    #    KIL3(2026-08-06)實測:真正沒過的是誤差 5.25 > 5.0,印出來的卻是
    #    「觸地 2/6」——分母寫死 6 造成的假陽性,而真因完全沒被列出。
    #    診斷訊息指錯方向比沒有訊息更貴。**每一個 ok 的條件都要有對應分支。**
    _why = []
    if n_t != _want_n:
        _why.append("觸地 %d/%d" % (n_t, _want_n))
    if mx is None:
        _why.append("誤差缺值")
    elif mx > miss_req:
        _why.append("誤差 %.2f > %.2f m" % (mx, miss_req))
    if spread > 3.0:
        _why.append("散布 %.2f > 3.00 s" % spread)
    if alt is None:
        _why.append("命中高度缺值")
    elif alt > 30.0:
        _why.append("命中高度 %.2f > 30 m" % alt)
    if gmean is None:
        _why.append("地速缺值")
    elif gmean < vmin_req:
        _why.append("地速 %.1f < %.1f m/s" % (gmean, vmin_req))
    if not first_pass_ok:
        _why.append("首航次全員命中" if isinstance(_v, dict) else "首航次判定缺席")
    if trial_rc != "0":
        _why.append("航次 rc=%s" % trial_rc)
    if _why:
        print("     ↳ 不通過的關鍵項:%s" % "、".join(_why))
PY
  python3 - "$SUMMARY" <<'PY'
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1]))]
if rows:
    p = sum(1 for r in rows if r["pass"] == "1")
    g = [float(r["gs_touch_mean"]) for r in rows if r.get("gs_touch_mean")]
    sp = [float(r["touch_spread_s"]) for r in rows if r.get("touch_spread_s")]
    print("  ▶▶ 累計 %d/%d 全過(不外推:n 這麼小只能當觀察值)" % (p, len(rows)))
    if g:
        print("     地速平均 %.1f m/s(各趟 %.1f – %.1f);散布 %.2f – %.2f s"
              % (sum(g) / len(g), min(g), max(g),
                 min(sp) if sp else 0.0, max(sp) if sp else 0.0))
PY
done
