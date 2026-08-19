#!/usr/bin/env python3
"""stot_extract_df.py — 從 ArduPilot dataflash(.BIN)抽出六機的完整航跡。

為什麼需要:`tot_node` 是**四散完才啟動**的,所以彈射起飛與巡航四散那兩段不在
`runs/tot_<TRIAL>.csv` 裡。要做「全程都是真實資料」的動畫就得回頭讀飛控自己的
dataflash —— 那是唯一從解鎖開始就有的記錄。

每台 uavN 的 log 編號不一樣(各自獨立計數),所以**用修改時間對應趟次**,不要用編號。

⚠️ 純 Python 的 DFReader 很慢,而且這支通常會和連跑批次同時執行 ——
   請一律用 `nice -n 19` 跑,不要跟 SITL 搶 CPU。

用法:
  nice -n 19 python3 stot_extract_df.py --trial S1 --out ~/fixwing_ws/runs/df_S1.csv
"""
import argparse
import csv
import glob
import math
import os
import sys
import time

from pymavlink import DFReader

EARTH = 111320.0
SR = os.path.expanduser("~/fixwing_ws/sitl_run")
RUNS = os.path.expanduser("~/fixwing_ws/runs")


def pick_log(uav, ref_mtime, tol=180.0, newest=False):
    """挑修改時間最接近該趟結束時刻的 .BIN。

    ⚠️ `--newest` 是給「tot_node 中途死掉」的航次用的:那時 `tot_<T>.csv` 的時間戳
       停在節點死亡的瞬間,而 dataflash 會一直寫到 SITL 停止 —— 實測差 1310 s,
       用時間吻合就會全部落空。只要那一趟是最後跑的,取最新的 log 才對。
    """
    cands = glob.glob(os.path.join(SR, f"uav{uav}", "logs", "*.BIN"))
    if not cands:
        return None, 1e18
    if newest:
        f = max(cands, key=os.path.getmtime)
        return f, abs(os.path.getmtime(f) - ref_mtime)
    best, bestd = None, 1e18
    for f in cands:
        d = abs(os.path.getmtime(f) - ref_mtime)
        if d < bestd:
            best, bestd = f, d
    return (best, bestd) if best and bestd <= tol else (None, bestd)


def extract(path, every, tmax_us):
    """回傳 [(t_boot_s, lat, lon, alt_rel, gs)]。

    ⚠️ 時間用 **TimeUS(飛控開機起算)**,不做每檔歸零。六台 SITL 幾乎同時開機,
       所以 TimeUS 是共同時間軸;各自歸零的話會把「第一筆 GPS 定位的時刻差」
       誤當成時間原點,實測會讓六機的時間軸互差 52 秒 —— 動畫就完全對不起來。
    ⚠️ 速度不要讀訊息欄位:`POS` 沒有速度欄(只有 `GPS` 有 `Spd`),而 POS 佔多數,
       直接讀會得到一整排 0。改用相鄰兩點的位移自己算。
    """
    m = DFReader.DFReader_binary(path)
    raw = []
    k = 0
    t0 = None
    home_alt = None
    while True:
        msg = m.recv_match(type=["POS", "GPS"])
        if msg is None:
            break
        if msg.get_type() == "GPS" and getattr(msg, "Status", 0) < 3:
            continue
        lat, lon, alt = msg.Lat, msg.Lng, msg.Alt
        if lat == 0 and lon == 0:
            continue
        tus = getattr(msg, "TimeUS", None)
        if tus is None:
            continue
        if t0 is None:
            t0, home_alt = tus, alt
        if tmax_us and tus - t0 > tmax_us:
            break
        k += 1
        if k % every:
            continue
        raw.append((tus / 1e6, lat, lon, alt - home_alt))

    out = []
    for j, (t, la, lo, z) in enumerate(raw):
        if j == 0:
            gs = 0.0
        else:
            pt, pla, plo, _ = raw[j - 1]
            dt = max(1e-3, t - pt)
            gs = math.hypot((la - pla) * EARTH,
                            (lo - plo) * EARTH * math.cos(math.radians(la))) / dt
        out.append((round(t, 2), la, lo, round(z, 1), round(min(gs, 400.0), 1)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trial", default="S1")
    ap.add_argument("--out", default=None)
    ap.add_argument("--every", type=int, default=20,
                    help="每 N 筆取樣一次(dataflash 很密,不抽會太大也太慢)")
    ap.add_argument("--max-min", type=float, default=0.0,
                    help="只讀前幾分鐘(0 = 全部)。彈射+四散大約前 6 分鐘")
    ap.add_argument("--uavs", default="1,2,3,4,5,6")
    ap.add_argument("--newest", action="store_true",
                    help="直接取各機最新的 log(該趟必須是最後跑的)。"
                         "tot_node 中途死掉的航次要用這個,見 pick_log 的說明")
    a = ap.parse_args()

    ref = os.path.join(RUNS, f"tot_{a.trial}.csv")
    if not os.path.exists(ref):
        sys.exit(f"找不到 {ref}")
    ref_m = os.path.getmtime(ref)
    out = a.out or os.path.join(RUNS, f"df_{a.trial}.csv")
    tmax = a.max_min * 60e6 if a.max_min else 0

    rows = []
    for u in [int(x) for x in a.uavs.split(",")]:
        f, d = pick_log(u, ref_m, newest=a.newest)
        if not f:
            print(f"  UAV{u}: 找不到時間吻合的 log(最近差 {d:.0f}s)", flush=True)
            continue
        t = time.time()
        seg = extract(f, a.every, tmax)
        print("  UAV%d %s  %d 點  (%.0f s 解析)"
              % (u, os.path.basename(f), len(seg), time.time() - t), flush=True)
        for s in seg:
            rows.append((s[0], u) + s[1:])

    # 六台共用同一個時間原點(全體最小的 TimeUS)
    if rows:
        base = min(r[0] for r in rows)
        rows = [(round(r[0] - base, 2),) + r[1:] for r in rows]
    rows.sort()
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "sysid", "lat", "lon", "alt", "gs"])
        w.writerows(rows)
    print("→ %s  (%d 列, %.1f MB)"
          % (out, len(rows), os.path.getsize(out) / 1048576))


if __name__ == "__main__":
    main()
