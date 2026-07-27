#!/usr/bin/env python3
"""散布是在哪一段長出來的?

對每一趟,算出六機各自「通過某個距目標半徑」的時刻,再看那個半徑上的時間散布。
散布若在交接距離(gate_r)之前就已經很大 → 問題在配速/錯開放行;
若在交接之後才長大 → 問題在 PN 段沒有時間對齊能力。
"""
import csv
import math
import sys
from collections import defaultdict

EARTH = 111320.0
RINGS = [2400, 2000, 1600, 1200, 1000, 800, 600, 400, 200, 100, 50]


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


for path in sys.argv[1:]:
    geom = path.replace(".csv", "_geom.csv")
    T = None
    try:
        with open(geom) as f:
            for r in csv.DictReader(f):
                if r["kind"] == "target":
                    T = (float(r["lat"]), float(r["lon"]))
    except FileNotFoundError:
        print("%s: 找不到 geom" % path)
        continue
    ml = mlon(T[0])

    tracks = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["phase"] != "DIVE":
                continue
            p = (float(r["lat"]), float(r["lon"]))
            d = math.hypot((p[0] - T[0]) * EARTH, (p[1] - T[1]) * ml)
            tracks[int(r["sysid"])].append((float(r["t"]), d, r["mode"]))

    # 每機通過各半徑的時刻(第一次由外往內穿過)
    cross = defaultdict(dict)
    for i, rows in tracks.items():
        for k in range(len(rows) - 1):
            d0, d1 = rows[k][1], rows[k + 1][1]
            for R in RINGS:
                if R in cross[i]:
                    continue
                if d0 > R >= d1:
                    f = (d0 - R) / (d0 - d1) if d0 != d1 else 0.0
                    cross[i][R] = rows[k][0] + f * (rows[k + 1][0] - rows[k][0])

    print("══ %s ══" % path.split("/")[-1])
    print("%-8s %8s %8s   %s" % ("半徑(m)", "散布(s)", "增量", "各機通過時刻"))
    prev = None
    for R in RINGS:
        ts = [cross[i][R] for i in sorted(cross) if R in cross[i]]
        if len(ts) < 6:
            print("%-8d %8s %8s   (只有 %d 架通過)" % (R, "-", "-", len(ts)))
            continue
        sp = max(ts) - min(ts)
        inc = "" if prev is None else "%+.2f" % (sp - prev)
        base = min(ts)
        rel = " ".join("%5.2f" % (t - base) for t in ts)
        print("%-8d %8.2f %8s   %s" % (R, sp, inc, rel))
        prev = sp
    print()
