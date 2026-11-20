#!/usr/bin/env bash
# make_artifacts.sh — 一趟航次結束後**立刻**產出進度報告用的視覺產物。
#
# ⚠️⚠️ 為什麼要「立刻」而不是事後補:本專案的航次資料是**覆寫式**的 ——
#   /tmp/trial_launch.log 只留最後一趟、cmdlog 舊版路徑寫死、dataflash 在
#   SITL 不重開時多趟共用同一份 .BIN。P91 的資料是靠開跑前手動備份才留下來的。
#   視覺產物如果不當場產生,下一趟一跑就永遠沒有了。
#
# ⚠️ 產物一趟一個資料夾,絕不互相覆蓋(這正是 cmdlog 踩過的坑)。
#
# 用法:
#   bash make_artifacts.sh <TRIAL>
#   ANIM=0 bash make_artifacts.sh <TRIAL>     # 跳過 GIF(快,只出靜態圖)
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
RUNS="$HOME/fixwing_ws/runs"
T="${1:?用法: bash make_artifacts.sh <TRIAL>}"
OUT="$RUNS/artifacts/$T"
mkdir -p "$OUT"

note() { printf '  %s\n' "$*"; }
ok=0; skip=0

# ⚠️ 每一項都獨立 try:某一支工具壞掉不可以讓整批產物沒有了。
#    (本輪已經有過「一個環節失敗導致整段靜默跳過」的教訓。)
try() {
  local name="$1"; shift
  if "$@" >"$OUT/.$name.log" 2>&1; then
    note "✓ $name"; ok=$((ok+1))
  else
    note "✗ $name(見 $OUT/.$name.log)"; skip=$((skip+1))
  fi
}

echo "══ 產出 $T 的進度報告素材 → $OUT ══"

# ── 1) 四項門檻摘要(純文字,最重要,先產)────────────────────────
{
  echo "# 航次 $T — 四項門檻"
  echo
  echo "## 門檻 1:彈射間隔 3 s"
  grep -E '彈射間隔|彈射 @' /tmp/trial_launch.log 2>/dev/null | tail -8 || echo "(無 launch log)"
  echo
  echo "## 門檻 2:巡航編隊"
  python3 "$SR/stot_stage2_report.py" "$T" 2>&1 | sed -n '1,14p' || true
  echo
  echo "## 門檻 3/4:命中精度與散布"
  grep -E '全員抵扇形|全員就緒|對準內切|全員命中|俯衝結案|判定 MISS|命中紅心 @|觸水終結' \
      "$RUNS/trial_$T.log" 2>/dev/null | sed -E 's/\[[^]]*\] \[[^]]*\]: //' || echo "(無 tot log)"
  echo
  echo "## 生效參數"
  grep -E '生效值|抵達判定' "$RUNS/trial_$T.log" 2>/dev/null | sed -E 's/.*\]: //' || true
} > "$OUT/summary.md" 2>/dev/null && note "✓ summary.md" || note "✗ summary.md"

# ── 2) 保存原始資料(它們會被下一趟覆寫)──────────────────────────
for f in "trial_$T.log" "tot_$T.csv" "tot_${T}_geom.csv" "show_$T.csv"; do
  [ -f "$RUNS/$f" ] && cp "$RUNS/$f" "$OUT/" 2>/dev/null
done
[ -d "$RUNS/cmdlog/$T" ] && cp -r "$RUNS/cmdlog/$T" "$OUT/cmdlog" 2>/dev/null
# ⚠️⚠️ /tmp/trial_launch.log **不帶航次編號,只留最後一趟**。回頭補素材時直接複製,
#    會把「最新那趟的起飛紀錄」冒名安到舊航次頭上(已經踩過:六趟補出一模一樣的
#    彈射間隔 3.01/3.02)。所以只在它確實屬於本航次時才收 —— 判準是它的時間戳必須
#    落在本航次的生命期內(起飛一定早於航次結束,且不會早於一小時前)。
if [ -f /tmp/trial_launch.log ] && [ -f "$RUNS/trial_$T.log" ]; then
  lt=$(stat -c %Y /tmp/trial_launch.log); tt=$(stat -c %Y "$RUNS/trial_$T.log")
  if [ "$lt" -le "$tt" ] && [ $((tt - lt)) -le 3600 ]; then
    cp /tmp/trial_launch.log "$OUT/launch.log" 2>/dev/null
  else
    echo "本航次的起飛紀錄已被後續航次覆寫,無法還原(/tmp/trial_launch.log 不分航次)。" \
        > "$OUT/launch.log.MISSING"
  fi
fi
note "✓ 原始資料已保存(trial log / tot csv / show csv / cmdlog / launch log)"

# ── 3) 靜態圖 ────────────────────────────────────────────────────
[ -f "$RUNS/tot_$T.csv" ] && [ -f "$RUNS/tot_${T}_geom.csv" ] && \
  try tot_plot python3 "$SR/_tot_plot.py" "$RUNS/tot_$T.csv" \
      "$RUNS/tot_${T}_geom.csv" "$OUT/${T}_tot_overview.png"

# ⚠️ _dive_plot.py 吃三個引數 <tot.csv> <geom.csv> <out.png>,少給會 IndexError。
[ -f "$RUNS/tot_$T.csv" ] && [ -f "$RUNS/tot_${T}_geom.csv" ] && \
  try dive_plot python3 "$SR/_dive_plot.py" "$RUNS/tot_$T.csv" \
      "$RUNS/tot_${T}_geom.csv" "$OUT/${T}_dive_profile.png"

# ⚠️ _formation_topview.py 吃的是 swarm_test_report.py **飛行中即時錄製**的 CSV
#    (要有 leader/pos 列標籤),不是 show_*.csv —— 餵錯會在 leader[-1] IndexError。
#    航次結束後那支已經產不出來了,所以用 _formation_from_show.py 從 show CSV 轉。
#    slot 幾何要跟 stot_stage2_report.py 同源,否則畫出來的 slot 位置對不上驗收數字。
if [ -f "$RUNS/show_$T.csv" ]; then
  VS="${V_SPACING:-4000}"; VA="${V_ANGLE:-35}"
  VT="${V_TAIL:-$(python3 -c "print($VS*1.5)")}"
  if python3 "$SR/_formation_from_show.py" "$RUNS/show_$T.csv" \
         "$OUT/${T}_formation.csv" >/dev/null 2>&1; then
    try formation python3 "$SR/_formation_topview.py" "$OUT/${T}_formation.csv" \
        "$OUT/${T}_formation_topview.png" "$VS" "$VA" "$VT"
  fi
fi

# ── 4) GIF 動畫 ──────────────────────────────────────────────────
if [ "${ANIM:-1}" = "1" ] && [ -f "$RUNS/tot_$T.csv" ]; then
  try mission_anim nice -n 19 python3 "$SR/stot_mission_anim.py" \
      --trial "$T" --out "$OUT/${T}_mission.gif"
fi

echo "── 產出 $ok 項,失敗/跳過 $skip 項 ──"
ls -la "$OUT" 2>/dev/null | grep -vE '^total|^d|\.log$' | awk '{printf "  %8s  %s\n", $5, $9}'
echo "路徑:$OUT"
