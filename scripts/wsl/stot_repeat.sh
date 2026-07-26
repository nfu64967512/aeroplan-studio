#!/usr/bin/env bash
# stot_repeat.sh — 同一組態連跑 N 趟,量成功率。
#
# 每趟都完整重啟 SITL(命中即消耗,機體留在地上,不重啟第二趟湊不齊六機)。
# 參數不在這裡定義 —— 一律透過環境變數傳給 stot_trial.sh,避免兩份參數各自漂移。
# 判定用 stot_score.py 的**觸地基準**(軌跡穿過 AGL=0 的那一點),不是「最近取樣點」。
#
# 用法:
#   PREFIX=P TRIALS=5 PN=True GATE_R=1200.0 DIVE_ANGLE=10.0 TRACK_FRAC=0.8 \
#     DIVE_SPDW=0.0 DIVE_PACE=50.0 LINE_TOL=25.0 HDEM_TC=0.5 DIVE_LAYER=4.0 \
#     bash stot_repeat.sh
#
# ⚠️ 不要 set -u:source ROS2 setup.bash 會踩到未設定的 AMENT_TRACE_SETUP_FILES。
set -o pipefail
SR="$HOME/fixwing_ws/sitl_run"
TRIALS="${TRIALS:-5}"
PREFIX="${PREFIX:-R}"
SUMMARY="${SUMMARY:-/tmp/stot_repeat.csv}"

[ -f "$SUMMARY" ] || echo "trial,n_touch,max_impact_miss_m,max_abs_alt_m,touch_spread_s,pass" > "$SUMMARY"

for n in $(seq 1 "$TRIALS"); do
  T="${PREFIX}${n}"
  echo "════════════════ REPEAT $T ════════════════"
  pkill -x tot_node 2>/dev/null
  bash "$SR/swarm_sitl_down.sh" >/dev/null 2>&1
  sleep 5
  # SITL 要有個常駐的父行程才不會被 PDEATHSIG 收掉 —— 本腳本就是那個父行程。
  ( cd "$SR" && JET=1 LAUNCH=catapult AIRSPEED_MAX_CAP=150 LOC=TAOYUAN \
      bash "$SR/swarm_sitl_up.sh" > "/tmp/repeat_sitl_${T}.log" 2>&1 ) &
  sleep 150
  if [ "$(pgrep -x arduplane | wc -l)" -ne 6 ]; then
    echo "$T,0,,,,0" >> "$SUMMARY"; echo "  ✗ SITL 起不來"; continue
  fi
  TRIAL="$T" bash "$SR/stot_trial.sh"
  python3 - "$T" "$SUMMARY" <<'PY'
import json, subprocess, sys, os
T, SUMMARY = sys.argv[1], sys.argv[2]
sr = os.path.expanduser("~/fixwing_ws/sitl_run")
try:
    out = subprocess.run(
        ["python3", os.path.join(sr, "stot_score.py"), f"/tmp/tot_{T}.csv",
         "--log", f"/tmp/trial_{T}.log", "--json"],
        capture_output=True, text=True, timeout=120).stdout
    s = json.loads(out)
except Exception as e:
    open(SUMMARY, "a").write(f"{T},0,,,,0\n")
    print(f"  ✗ 計分失敗:{e}")
    raise SystemExit
per = s["per_uav"]
touch = [v for v in per.values() if v.get("impact_miss_m") is not None]
n_t = len(touch)
mx = max((v["impact_miss_m"] for v in touch), default=None)
alt = max((abs(v["alt_agl_m"]) for v in per.values()
           if v.get("alt_agl_m") is not None), default=None)
ts = [v["impact_t_s"] for v in touch]
spread = (max(ts) - min(ts)) if len(ts) > 1 else 0.0
ok = int(n_t == 6 and mx is not None and mx <= 5.0 and spread <= 3.0
         and alt is not None and alt <= 30.0)
open(SUMMARY, "a").write(
    "%s,%d,%s,%s,%.2f,%d\n" % (T, n_t, "" if mx is None else "%.2f" % mx,
                               "" if alt is None else "%.2f" % alt, spread, ok))
print("  ▶ %s 觸地 %d/6  誤差 max %s m  最低AGL max %s m  散布 %.2f s  → %s"
      % (T, n_t, mx, alt, spread, "✅" if ok else "❌"))
PY
  python3 - "$SUMMARY" <<'PY'
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1]))]
if rows:
    p = sum(1 for r in rows if r["pass"] == "1")
    print("  ▶▶ 累計 %d/%d 全過(不外推:n 這麼小只能當觀察值)" % (p, len(rows)))
PY
done
