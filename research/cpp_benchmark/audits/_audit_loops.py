from pathlib import Path
# -*- coding: utf-8 -*-
"""拆開螺旋本體：每一圈的長度 vs 它「需要」的長度（面積/pitch），找浪費在哪。"""
import math, sys
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from bench import rect_xy
import algo_spiral as SP

SW = 2*140.0*math.tan(math.radians(60)/2); PI = SW*0.7
L_SHAPE = Polygon([(-2500,-2000),(2500,-2000),(2500,0),(0,0),(0,2000),(-2500,2000)])
CASES = [("(1) rect", [rect_xy(0,0,2500,2000)]),
         ("(2) 4 narrow", [rect_xy(cx,0,625,2000) for cx in (-2625,-875,875,2625)]),
         ("(3) L", [L_SHAPE])]

for title, polys in CASES:
    area = unary_union(polys)
    geoms = SP._polys(area)
    loops, cur = [], (0.0,0.0)
    for g in geoms:
        lp, cur = SP._spiral_loops(g, SW, PI, cur)
        loops += lp
    print(f"\n{title}: 圈數 {len(loops)}")
    thin_len = 0.0; thin_n = 0
    for i, lp in enumerate(loops):
        L = LineString(lp)
        ring = Polygon(lp) if len(lp) >= 4 else None
        a = ring.area if ring and ring.is_valid else 0.0
        # 「細」判準：圈圍出的面積小到相當於一條線（等效寬度 < swath）
        eqw = 2*a/L.length if L.length > 0 else 0.0
        flag = "  <== 細長,等於來回走同一條線" if eqw < SW*0.5 else ""
        if eqw < SW*0.5:
            thin_len += L.length; thin_n += 1
        print(f"  圈{i:2d} 長 {L.length:8.1f} m  圍面積 {a:11.1f} m²  等效寬 {eqw:7.1f} m{flag}")
    print(f"  細長圈 {thin_n} 條、總長 {thin_len/1000:.2f} km"
          f"（理論上只需一半 => 可省 {thin_len/2000:.2f} km）")
