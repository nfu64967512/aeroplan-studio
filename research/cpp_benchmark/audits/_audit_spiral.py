from pathlib import Path
# -*- coding: utf-8 -*-
"""審查 algo_spiral：健全性 + 對抗性檢定。"""
import math, sys
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from bench import metrics, rect_xy, boustrophedon, min_width_angle, _strips
import algo_spiral as SP

ALT, HFOV = 140.0, 60.0
SW = 2 * ALT * math.tan(math.radians(HFOV) / 2)
PI = SW * 0.7
L_SHAPE = Polygon([(-2500,-2000),(2500,-2000),(2500,0),(0,0),(0,2000),(-2500,2000)])
CASES = [("(1) rect 5x4km n=8", [rect_xy(0,0,2500,2000)], 8),
         ("(2) 4 strips n=8", [rect_xy(cx,0,625,2000) for cx in (-2625,-875,875,2625)], 8),
         ("(3) L-shape n=4", [L_SHAPE], 4)]

print("=== A. 結構健全性 ===")
for name, polys, n in CASES:
    tr = SP.plan(polys, n, swath_m=SW, pitch_m=PI)
    ok_start = all(t[0]==(0.0,0.0) and t[-1]==(0.0,0.0) for t in tr)
    fin = all(all(math.isfinite(c) for p in t for c in p) for t in tr)
    print(f"{name}: n={len(tr)}/{n} start/end_ok={ok_start} finite={fin} "
          f"pts={[len(t) for t in tr]}")

print("\n=== B. 切片有沒有漏路徑（n=1 掃到 vs n=k 掃到）===")
for name, polys, n in CASES:
    t1 = SP.plan(polys, 1, swath_m=SW, pitch_m=PI)
    tn = SP.plan(polys, n, swath_m=SW, pitch_m=PI)
    s1 = unary_union([LineString(t).buffer(SW/2, quad_segs=8) for t in t1])
    sn = unary_union([LineString(t).buffer(SW/2, quad_segs=8) for t in tn])
    area = unary_union(polys)
    lost = (s1.difference(sn)).intersection(area).area
    print(f"{name}: n=1 掃到但 n={n} 漏掉 = {lost:.1f} m²  (作業區 {area.area:,.0f})")

print("\n=== C. 螺旋本體長度 vs 「pitch 理想」(area/pitch) ===")
for name, polys, n in CASES:
    dbg, tr = SP.plan(polys, n, swath_m=SW, pitch_m=PI, debug=True)
    A = unary_union(polys).area
    ideal_pitch = A / PI / 1000.0
    ideal_swath = A / SW / 1000.0
    body = dbg['spiral_km']
    tot = sum(sum(math.dist(t[i],t[i-1]) for i in range(1,len(t))) for t in tr)/1000
    print(f"{name}: 本體 {body:.2f} km  pitch理想 {ideal_pitch:.2f}  "
          f"本體/pitch理想 {body/ideal_pitch:.3f}  轉場+去回 {tot-body:.2f} km")
