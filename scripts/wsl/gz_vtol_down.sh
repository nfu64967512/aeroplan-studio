#!/usr/bin/env bash
# gz_vtol_down.sh — 只收掉 **gz_vtol_up.sh 這一趟自己起的** gz sim 與 arduplane。
#
# ═══════════════════════════════════════════════════════════════════════════
# ⚠️⚠️⚠️ 這支腳本**絕對不會**做 `pkill -9 -f "gz sim"` 之類的事
# ═══════════════════════════════════════════════════════════════════════════
# 本專案有 pkill 自殺與誤殺的前科(pkill 的樣式會咬到自己的命令列;`-f "gz sim"`
# 會連別人正在跑的航次一起殺掉 —— 撰寫本檔時,同一台機器上就有另一個 session
# 的四機 Gazebo(gz server + 4×arducopter)在跑)。
#
# 指認方式(由硬到軟):
#   ① **/proc/<pid>/environ 裡有 `GZ_VTOL_RUN_ID=<本趟的 id>`** ← 判決依據
#      這是最硬的指認:環境變數在 exec 時就固定了,改不掉、偽裝不了,而且
#      和行程名、命令列、cwd 全都無關。gz sim 是 ruby wrapper 再 fork 出
#      `gz sim server` / `gz sim gui`,用名字去比一定會漏;環境變數會被繼承,
#      所以整棵子樹都抓得到。
#   ② state 檔裡記下的 pid —— 只拿來**對帳**(有沒有漏、有沒有多),不當判決。
#      pid 會被回收,單靠 pid 有殺錯人的風險。
#
# 三道保險,絕不誤殺:
#   * 跳過自己($$)與**自己所有的祖先**(萬一有人 `source` 了 up.sh,
#     終端本身也會帶著同一個 RUN_ID)
#   * 跳過 pid 1
#   * 先 SIGTERM、等、再對倖存者 SIGKILL(不是一上來就 -9)
#
# 收完之後會**用 gz_port_probe.py 讀回**:同一批埠必須回到「可獨占 bind」。
# 「送了 SIGKILL」不等於「埠放掉了」—— 這一步才是關掉的證據。
#
# ═══════════════════════════════════════════════════════════════════════════
# 用法
# ═══════════════════════════════════════════════════════════════════════════
#   bash gz_vtol_down.sh                     # 收掉 state 檔記的那一趟
#   DRY_RUN=1 bash gz_vtol_down.sh           # 只列出會殺誰,不動手
#   RUN_ID=vtol-20260909-... bash gz_vtol_down.sh   # state 檔不見時指定
#   LIST=1 bash gz_vtol_down.sh              # 列出機器上所有 vtol 趟次的 RUN_ID
#   RUN=/path/to/gz_run bash gz_vtol_down.sh # 換工作目錄
#
# 離開碼:0 收乾淨(**而且埠位讀回驗過**)/ 1 還有殘留(埠沒放掉)
#        2 用法或找不到趟次 / 5 訊號送出去了但**沒有驗證過**
# ⚠️⚠️ 5 是後補的。第一版在「找不到 probe」或「state 沒有埠位資訊」時只印一行
#   warn 就 exit 0 ⇒ 呼叫端拿到的是「收乾淨」這個**沒有被檢查過的斷言**。
#   本專案的硬性要求是不接受靜默失敗,而檔頭自己也寫著「送了 SIGKILL 不等於
#   埠放掉了 —— 這一步才是關掉的證據」。沒有證據就不可以回 0。
set -uo pipefail

SR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="$SR/gz_port_probe.py"
RUN="${RUN:-$HOME/vtol_ws/gz_run}"
STATE="$RUN/vtol_swarm.env"
TERM_WAIT_S="${TERM_WAIT_S:-8}"
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '[down] %s\n' "$*"; }
warn() { printf '[down] ⚠️  %s\n' "$*" >&2; }
# 訊息只取 $1:第二個位置參數是**離開碼**,用 "$*" 會把它一起印進訊息。
#    現在還沒有帶碼的 die 呼叫,但那是等著被踩的地雷(up.sh 就踩到了)。
die()  { printf '[down] ✗ %s\n' "$1" >&2; exit "${2:-2}"; }

# ── 掃出機器上所有帶 GZ_VTOL_RUN_ID 的行程 ────────────────────────────────
# 讀不到 environ 的(別的使用者、已消失)直接跳過 —— 我們寧可漏殺也不誤殺。
scan_ids() {
  local pid val
  for pid in /proc/[0-9]*; do
    pid="${pid##*/}"
    [ -r "/proc/$pid/environ" ] || continue
    val="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
           | grep -m1 '^GZ_VTOL_RUN_ID=' | cut -d= -f2-)"
    [ -n "$val" ] && printf '%s\n' "$val"
  done | sort -u
}

pids_of_run() {   # pids_of_run <run_id>
  local want="$1" pid
  for pid in /proc/[0-9]*; do
    pid="${pid##*/}"
    [ -r "/proc/$pid/environ" ] || continue
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
         | grep -qxF "GZ_VTOL_RUN_ID=$want"; then
      printf '%s\n' "$pid"
    fi
  done
}

# ⚠️ 先確認可讀再開檔:`< /proc/<pid>/cmdline` 的重導向失敗是 **shell** 層級的錯誤,
#    tr 的 2>/dev/null 擋不到,會在畫面上噴一行 "No such file or directory"。
#    行程在掃描到讀取之間消失是常態(實測 T8 撞到),不該長得像故障。
cmd_of() {
  [ -r "/proc/$1/cmdline" ] || { printf '(行程已結束)'; return 0; }
  tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | cut -c1-140
}

if [ "${LIST:-0}" = "1" ]; then
  say "機器上帶 GZ_VTOL_RUN_ID 的趟次:"
  ids="$(scan_ids)"
  [ -n "$ids" ] || { say "  (一個都沒有)"; exit 0; }
  while read -r id; do
    [ -n "$id" ] || continue
    n="$(pids_of_run "$id" | wc -l)"
    say "  $id   行程數=$n"
  done <<<"$ids"
  exit 0
fi

# ── 決定要收哪一趟 ────────────────────────────────────────────────────────
RUN_ID="${RUN_ID:-}"
if [ -z "$RUN_ID" ] && [ -f "$STATE" ]; then
  # shellcheck disable=SC1090
  . "$STATE"
  RUN_ID="${GZ_VTOL_RUN_ID:-}"
  say "讀 state:$STATE"
fi
if [ -z "$RUN_ID" ]; then
  ids="$(scan_ids)"
  n="$(printf '%s' "$ids" | grep -c . || true)"
  if [ "${n:-0}" = "1" ]; then
    RUN_ID="$ids"
    warn "找不到 state 檔,但機器上只有一趟($RUN_ID),就收它。"
  else
    # ⚠️ 這裡的示範指令**不可以**用反引號括起來:在雙引號字串裡反引號是命令替換,
    #    寫成 `LIST=1 bash gz_vtol_down.sh` 會讓「印一段說明」變成「真的再跑一次自己」。
    #    (實測抓到的,原本就是這一行。)
    die "指不出要收哪一趟(state 檔 $STATE 不在,而機器上有 ${n:-0} 趟)。
     先跑 'LIST=1 bash gz_vtol_down.sh' 看清單,再用 RUN_ID=... 指定。
     ⚠️ 刻意**不**提供『全部殺掉』的預設 —— 這台機器上會有別人的航次。"
  fi
fi
say "本次要收的 RUN_ID = $RUN_ID"

# ── 保險:自己與自己的祖先絕不入列 ────────────────────────────────────────
SAFE=" $$ 1 "
p="$$"
while [ -n "$p" ] && [ "$p" != "0" ] && [ "$p" != "1" ]; do
  p="$(awk '/^PPid:/{print $2}' "/proc/$p/status" 2>/dev/null)"
  [ -n "$p" ] || break
  SAFE="$SAFE$p "
done
say "保護名單(自己與祖先):$SAFE"

# ⚠️ 光有「自己與祖先」還不夠:本腳本自己的 environ 也可能帶著同一個 RUN_ID
#    (從 up.sh 的環境裡跑、或有人 source 過),那麼它**自己 fork 出來的子行程**
#    —— $(cmd_of ...) 的命令替換、mapfile 的行程替換 —— 也會被掃進來。
#    實測 T6 就抓到一個這樣的暫態 pid(cmdline 已經空了)。
#    ⇒ 再加一道:凡是往上走會走到 $$ 的,都是我們自己的子孫,一律跳過。
is_my_descendant() {
  local p="$1" hops=0
  while [ -n "$p" ] && [ "$p" != "0" ] && [ "$p" != "1" ] && [ "$hops" -lt 64 ]; do
    p="$(awk '/^PPid:/{print $2}' "/proc/$p/status" 2>/dev/null)"
    [ -n "$p" ] || return 1
    [ "$p" = "$$" ] && return 0
    hops=$((hops+1))
  done
  return 1
}

mapfile -t VICTIMS < <(pids_of_run "$RUN_ID")
KEEP=()
for pid in "${VICTIMS[@]:-}"; do
  [ -n "$pid" ] || continue
  case "$SAFE" in *" $pid "*) warn "跳過 pid $pid(自己或祖先)"; continue ;; esac
  # 掃描到這裡之間行程可能已經結束(命令替換的暫態子行程就是這樣)。
  # /proc/<pid>/status 不在 = 它已經走了,沒有列出來也沒有送訊號的必要。
  [ -r "/proc/$pid/status" ] || continue
  if is_my_descendant "$pid"; then warn "跳過 pid $pid(本腳本自己的子行程)"; continue; fi
  KEEP+=("$pid")
done

if [ "${#KEEP[@]}" -eq 0 ]; then
  say "沒有找到屬於 $RUN_ID 的行程 —— 可能已經收乾淨了。"
else
  say "要收掉 ${#KEEP[@]} 個行程:"
  for pid in "${KEEP[@]}"; do say "  pid $pid  $(cmd_of "$pid")"; done
fi

# ── state 檔對帳(只是對帳,不當判決)──────────────────────────────────────
if [ -n "${VTOL_GZ_PID:-}" ]; then
  # ⚠️ 0 是 state 檔裡「當時沒抓到 pid」的哨兵值,**不可以**丟給 kill -0:
  #    `kill -0 0` 打的是**整個 process group**,一定成功 ⇒ 會印出
  #    「state 記著 pid 0 還活著」這種毫無意義又嚇人的訊息(實測 T7 撞到)。
  recorded=()
  [ "${VTOL_GZ_PID:-0}" -gt 0 ] 2>/dev/null && recorded+=("$VTOL_GZ_PID")
  for k in $(seq 0 $(( ${VTOL_N:-0} - 1 ))); do
    eval "v=\${VTOL_SITL_PID_$k:-}"
    [ -n "${v:-}" ] && [ "${v:-0}" -gt 0 ] 2>/dev/null && recorded+=("$v")
  done
  for r in "${recorded[@]:-}"; do
    [ -n "$r" ] || continue
    case " ${KEEP[*]:-} " in
      *" $r "*) ;;
      *) if kill -0 "$r" 2>/dev/null; then
           warn "state 記著 pid $r 還活著,但它的 environ 裡沒有本趟的 RUN_ID
     ⇒ **不動它**。可能是 pid 被回收給別人了(這正是不能只靠 pid 的原因):
        $(cmd_of "$r")"
         fi ;;
    esac
  done
fi

if [ "$DRY_RUN" = "1" ]; then
  say "DRY_RUN=1:到此為止,沒有送出任何訊號。"
  exit 0
fi

# ── SIGTERM → 等 → SIGKILL ────────────────────────────────────────────────
if [ "${#KEEP[@]}" -gt 0 ]; then
  for pid in "${KEEP[@]}"; do kill -TERM "$pid" 2>/dev/null; done
  say "已送 SIGTERM,最多等 ${TERM_WAIT_S}s…"
  deadline=$(( $(date +%s) + TERM_WAIT_S ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    alive=0
    for pid in "${KEEP[@]}"; do kill -0 "$pid" 2>/dev/null && alive=$((alive+1)); done
    [ "$alive" -eq 0 ] && break
    sleep 0.5
  done
  for pid in "${KEEP[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      warn "pid $pid 不理 SIGTERM,送 SIGKILL:$(cmd_of "$pid")"
      kill -KILL "$pid" 2>/dev/null
    fi
  done
  sleep 1
fi

# ── 讀回:埠真的放掉了嗎 ──────────────────────────────────────────────────
# 「送了訊號」不是證據。⚠️ 而且這裡**不能**只看 ss:別人的 gz 也是
# SO_REUSEADDR,ss 上長得一模一樣。用 probe 的獨占 bind 才分得出來。
# rc 的初值刻意是 5(未驗證),只有 probe 真的跑完並回 0 才降成 0。
# 這樣「忘了驗」與「驗過是乾淨的」在離開碼上分得開。
rc=5
# ⚠️ 這裡刻意把「檔案不見」和「state 沒有埠位資訊」分開講。第一版兩個併成一句
#    「(或找不到 <path>)」,實測跑出來時 probe 明明在,訊息卻指向它 ——
#    misattribution 會驅動錯誤的修復,是本專案列管的那一類問題。
if [ ! -f "$PROBE" ]; then
  warn "找不到 $PROBE ⇒ 跳過埠位讀回;這一趟的『收乾淨』**沒有被驗證過**(離開碼 5)。"
elif [ -z "${VTOL_INSTANCES:-}" ] || [ -z "${VTOL_TCP_BASE:-}" ]; then
  warn "state($STATE)裡沒有 VTOL_INSTANCES / VTOL_TCP_BASE ⇒ 不知道該驗哪些埠;
     這一趟的『收乾淨』**沒有被驗證過**(離開碼 5)。
     (用 RUN_ID= 手動指定趟次時常會這樣;要驗就把 VTOL_INSTANCES/VTOL_TCP_BASE
      補進環境,或直接跑 gz_port_probe.py。)"
else
  say "讀回:同一批埠是否已釋放(獨占 bind,不是看 ss)…"
  python3 "$PROBE" --instances "$(printf '%s' "$VTOL_INSTANCES" | tr ' ' ',')" \
          --tcp-base "$VTOL_TCP_BASE" >"$RUN/down_probe.log" 2>&1
  rc=$?
  if [ "$rc" -eq 0 ]; then
    say "✔ 埠全部釋放(見 $RUN/down_probe.log)"
  elif [ "$rc" -eq 2 ]; then
    # ⚠️ probe 的 rc=2 是**它的組態**有問題(埠算出來超範圍等),不是埠沒放掉。
    #    照著「還有埠沒放掉」去查會查到一個不存在的殘留行程 —— 錯誤歸因。
    warn "埠位讀回沒跑成:probe 回 rc=2(組態問題,不是殘留)。細節:"
    tail -20 "$RUN/down_probe.log" >&2
    rc=5
  else
    warn "還有埠沒放掉 —— 細節:"
    grep -E '^\s*\[[✗!]\]|└─' "$RUN/down_probe.log" >&2 || tail -20 "$RUN/down_probe.log" >&2
    warn "⚠️ 也可能是**別人**在我們放手之後立刻補上了那個埠。上面的 ss 指認會說是誰;
     如果是別的 session,那不是殘留,是他們的航次 —— 不要去殺。"
  fi
fi

# 殘留檔案不刪:log 是下一趟出事時唯一的證據,而且本專案的教訓是
# 「資料會被下一趟覆寫」,所以這裡連清都不清,交給呼叫者自己歸檔。
say "log 保留在 $RUN/(gz.log、uav*/sitl.log)—— 不自動刪,出事時那是唯一的證據。"
exit "$rc"
