from pathlib import Path
# -*- coding: utf-8 -*-
"""對抗性檢定：把 spiral 自己的 DP minimax 分配器裝到牛耕上，看優勢還剩多少。

他自述只修了 bench 牛耕的「7 台」bug（等長切），但**沒有**把自己的 DP 分配
借給對手。分配器是他優勢的一大來源（不均 1.013 vs 1.137），必須拆開量。
"""
import math, sys
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely.geometry import Polygon
from shapely.ops import unary_union
from bench import metrics, rect_xy, boustrophedon, min_width_angle, _strips
import algo_spiral as SP

ALT, HFOV = 140.0, 60.0
SW = 2 * ALT * math.tan(math.radians(HFOV)/2); PI = SW*0.7
L_SHAPE = Polygon([(-2500,-2000),(2500,-2000),(2500,0),(0,0),(0,2000),(-2500,2000)])


def bous_dp(polys, n_uav, *, swath_m, pitch_m, start=(0.0,0.0)):
    """牛耕條帶 + spiral 那顆 DP minimax 分配器（連接段標成可丟的轉場）。"""
    segs = []
    for p in polys:
        segs += _strips(p, pitch_m, min_width_angle(p))
    if not segs:
        return []
    atoms, flip = [], False
    for a, b in segs:
        atoms.append([b, a] if flip else [a, b])
        flip = not flip
    pts, dead, cum = SP._build_polyline(atoms)
    return SP._partition(pts, dead, cum, (float(start[0]), float(start[1])), n_uav)


def bous_fixed(polys, n_uav, *, swath_m, pitch_m, start=(0.0,0.0)):
    """只修「7 台」bug（絕對門檻），分配仍是等長切 —— 他自述的那份對照。"""
    segs = []
    for p in polys:
        segs += _strips(p, pitch_m, min_width_angle(p))
    if not segs:
        return []
    lens = [math.dist(a,b) for a,b in segs]
    share = sum(lens)/max(n_uav,1)
    tracks, cur, acc, flip, k = [], [start], 0.0, False, 1
    for (a,b), L in zip(segs, lens):
        p0,p1 = (b,a) if flip else (a,b)
        cur += [p0,p1]; flip = not flip; acc += L
        if acc >= share*k and len(tracks) < n_uav-1:
            cur.append(start); tracks.append(cur)
            cur, flip = [start], False; k += 1
    cur.append(start); tracks.append(cur)
    return [t for t in tracks if len(t) >= 2]


CASES = [("(1) rect 5x4km", [rect_xy(0,0,2500,2000)], 8),
         ("(2) 4 narrow", [rect_xy(cx,0,625,2000) for cx in (-2625,-875,875,2625)], 8),
         ("(3) L-shape", [L_SHAPE], 4)]
hdr = f"{'方法':22s}{'台':>3s}{'總km':>8s}{'最長km':>8s}{'覆蓋':>8s}{'開銷':>7s}{'重複':>7s}{'不均':>7s}{'掃描率':>9s}{'浪費':>8s}"
def show(nm, m):
    print(f"{nm:22s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:8.2f}"
          f"{m['coverage']*100:7.2f}%{m['overhead'] or 0:7.3f}{m['redundancy'] or 0:7.3f}"
          f"{m['balance'] or 0:7.3f}{m['scan_rate']:9.2f}{(m['waste'] or 0)*100:7.2f}%")

for title, polys, n in CASES:
    print(f"\n{title}  n={n}"); print(hdr)
    sp = metrics(polys, SP.plan(polys,n,swath_m=SW,pitch_m=PI), SW)
    b0 = metrics(polys, boustrophedon(polys,n,swath_m=SW,pitch_m=PI), SW)
    b1 = metrics(polys, bous_fixed(polys,n,swath_m=SW,pitch_m=PI), SW)
    b2 = metrics(polys, bous_dp(polys,n,swath_m=SW,pitch_m=PI), SW)
    show("spiral", sp); show("boustro (bench 原版)", b0)
    show("boustro (他的修正版)", b1); show("boustro + 同一顆DP", b2)
    print(f"  掃描率優勢 vs 他的修正版 {sp['scan_rate']/b1['scan_rate']-1:+.1%}"
          f"   vs 同一顆DP {sp['scan_rate']/b2['scan_rate']-1:+.1%}")
