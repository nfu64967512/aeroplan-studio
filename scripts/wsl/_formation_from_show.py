#!/usr/bin/env python3
"""_formation_from_show.py — 把 show_<T>.csv 轉成 _formation_topview.py 吃的格式。

為什麼需要這支:編隊俯視圖原本的資料源是 swarm_test_report.py,而那支是
**飛行中即時訂閱 ROS2 topic 錄製**的 —— 航次結束後就再也產不出來。但同樣的
原始資料 show_<T>.csv 一直都有(t,sysid,lat,lon,alt,phase),差的只是:
  ① 沒有 leader/pos 的列標籤    ② 沒有長機 heading(topview 要拿它建座標框)
heading 用長機連續兩點的地面航跡角補(作圖用途,精度綽綽有餘)。

用法: python3 _formation_from_show.py <show.csv> <out.csv> [leader_sysid]
輸出列格式(對齊 swarm_test_report.py:249 的既有 schema):
    leader,t,id,lat,lon,alt,hdg
    pos,t,id,lat,lon
"""
import csv
import math
import sys

SRC, DST = sys.argv[1], sys.argv[2]
LEAD = int(sys.argv[3]) if len(sys.argv) > 3 else 1

rows = []
with open(SRC, newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["t"]), int(r["sysid"]),
                         float(r["lat"]), float(r["lon"]), float(r["alt"])))
        except (KeyError, ValueError):
            continue          # 壞列跳過,不要讓整趟的圖沒有了
rows.sort(key=lambda x: x[0])

lead = [r for r in rows if r[1] == LEAD]
if not lead:
    sys.exit(f"show CSV 裡找不到 sysid={LEAD} 的長機樣本")


def bearing(a, b):
    """a→b 的地面航跡角(度,北為 0)。"""
    la1, lo1, la2, lo2 = map(math.radians, (a[2], a[3], b[2], b[3]))
    dlo = lo2 - lo1
    y = math.sin(dlo) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlo)
    return math.degrees(math.atan2(y, x)) % 360.0


# heading 用前後點差分;位移過小(<1 m)時沿用上一個值,免得原地抖動亂算角度。
hdg, prev = [], 0.0
for i, r in enumerate(lead):
    nxt = lead[min(i + 1, len(lead) - 1)]
    d = math.hypot((nxt[2] - r[2]) * 111320.0,
                   (nxt[3] - r[3]) * 111320.0 * math.cos(math.radians(r[2])))
    prev = bearing(r, nxt) if d > 1.0 else prev
    hdg.append(prev)

with open(DST, "w", newline="") as f:
    w = csv.writer(f)
    for r, h in zip(lead, hdg):
        w.writerow(["leader", f"{r[0]:.2f}", r[1], f"{r[2]:.7f}",
                    f"{r[3]:.7f}", f"{r[4]:.1f}", f"{h:.1f}"])
    for r in rows:
        if r[1] != LEAD:
            w.writerow(["pos", f"{r[0]:.2f}", r[1], f"{r[2]:.7f}", f"{r[3]:.7f}"])

print(f"{len(lead)} 筆長機 + {len(rows) - len(lead)} 筆僚機 → {DST}")
