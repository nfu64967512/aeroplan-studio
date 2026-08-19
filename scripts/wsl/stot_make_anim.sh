#!/usr/bin/env bash
# stot_make_anim.sh — 一鍵把某一趟做成動畫(抽 dataflash → 出 GIF → 出代表影格)。
#
# 每一版迭代都要留動畫,**不管那一趟成功或失敗** —— 失敗的那幾版才看得出問題在哪,
# 做進度報告時是最有用的素材。
#
# 用法:
#   bash stot_make_anim.sh S1                 # 用時間戳對應 log
#   NEWEST=1 bash stot_make_anim.sh T1        # 該趟是最後跑的(tot_node 中途掛掉時用)
#   OUT_DIR=~/shots bash stot_make_anim.sh S1
#
# ⚠️ 跑在 nice -n 19:這支常常和 SITL 連跑同時執行,不要跟飛控搶 CPU。
set -o pipefail
T="${1:?用法: bash stot_make_anim.sh <TRIAL>}"
SR="$HOME/fixwing_ws/sitl_run"
RUNS="$HOME/fixwing_ws/runs"
OUT_DIR="${OUT_DIR:-$RUNS/figs}"
mkdir -p "$OUT_DIR"

EXTRA=""
[ "${NEWEST:-0}" = "1" ] && EXTRA="--newest"

echo "── 抽 dataflash(全程真實航跡)──"
nice -n 19 python3 "$SR/stot_extract_df.py" --trial "$T" --every "${EVERY:-12}" $EXTRA \
  || exit 1

echo "── 出動畫 ──"
nice -n 19 python3 "$SR/stot_mission_anim.py" --trial "$T" --auto-phase \
  --out "$OUT_DIR/mission_${T}.gif" || exit 1

echo "── 代表影格 ──"
nice -n 19 python3 - "$OUT_DIR/mission_${T}.gif" "$OUT_DIR/frames_${T}.png" <<'PY'
import sys
from PIL import Image
im = Image.open(sys.argv[1])
n = im.n_frames
picks = [int(n * f) for f in (0.10, 0.28, 0.45, 0.62, 0.80, 0.97)]
tiles = []
for p in picks:
    im.seek(min(p, n - 1))
    tiles.append(im.convert("RGB").copy())
w, h = tiles[0].size
w2, h2 = int(w * 0.5), int(h * 0.5)
sheet = Image.new("RGB", (w2 * 2, h2 * 3), "white")
for k, t in enumerate(tiles):
    sheet.paste(t.resize((w2, h2)), ((k % 2) * w2, (k // 2) * h2))
sheet.save(sys.argv[2])
print(sys.argv[2], "frames", n)
PY
ls -la "$OUT_DIR/mission_${T}.gif" "$OUT_DIR/frames_${T}.png"
