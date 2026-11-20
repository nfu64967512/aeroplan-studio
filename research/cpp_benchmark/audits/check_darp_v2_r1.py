from pathlib import Path
"""algo_darp_v2_r1 的契約檢查 + 對 algo_darp / boustrophedon 的隨機回歸掃描。"""
import math, random, sys, time
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely import affinity
from shapely.geometry import MultiPoint, Polygon
from bench import boustrophedon, metrics, rect_xy
from algo_darp import plan as v1
from algo_darp_v2_r1 import plan as v2

SW = 2*140.0*math.tan(math.radians(60.0)/2); P = SW*0.7
fails = []

def case(name, polys, n, start=(0.0,0.0)):
    t0=time.perf_counter(); tr = v2(polys, n, swath_m=SW, pitch_m=P, start=start)
    dt=time.perf_counter()-t0
    for i,t in enumerate(tr):
        if tuple(t[0])!=start or tuple(t[-1])!=start:
            fails.append(f"{name}: track{i} 頭尾不在 start")
        if any(not math.isfinite(c) for p in t for c in p):
            fails.append(f"{name}: track{i} 非有限座標")
    m  = metrics(polys, tr, SW) if tr else {"coverage":0,"scan_rate":0,"n_uav":0,"max_km":0}
    m1 = metrics(polys, v1(polys,n,swath_m=SW,pitch_m=P,start=start), SW)
    mb = metrics(polys, boustrophedon(polys,n,swath_m=SW,pitch_m=P,start=start), SW)
    if tr and m["coverage"] < 0.99: fails.append(f"{name}: cov {m['coverage']:.4f}<0.99")
    if tr and len(tr) != n:        fails.append(f"{name}: 回 {len(tr)} 條，要 {n}")
    if tr and m["scan_rate"] < m1["scan_rate"] - 1e-6:
        fails.append(f"{name}: v2 {m['scan_rate']:.2f} 比 v1 {m1['scan_rate']:.2f} 差")
    print(f"{name:26s} n={n:<2d} 台={m['n_uav']:<2d} cov={m['coverage']*100:6.2f}% "
          f"max={m['max_km']:7.2f}  掃描率 v2={m['scan_rate']:6.2f} v1={m1['scan_rate']:6.2f} "
          f"bous={mb['scan_rate']:6.2f}  v2/v1={m['scan_rate']/max(m1['scan_rate'],1e-9):5.3f} {dt:5.1f}s")
    return m, m1, mb

print("── 契約 / 退化輸入 ─────────────────────────────────────────────")
rect=[rect_xy(0,0,2500,2000)]
for n in (1,2,4,8,16): case("矩形 5x4", rect, n)
case("非原點 start", rect, 4, start=(-9000.0,-7000.0))
case("非原點 start2", rect, 4, start=(2500.0,2000.0))
case("小於刈幅", [rect_xy(1000,1000,60,40)], 3)
case("細長帶 6000x90", [rect_xy(0,0,3000,45)], 4)
case("兩塊互相重疊", [rect_xy(0,0,1500,1500), rect_xy(1000,1000,1500,1500)], 4)
print("空清單 ->", v2([],4,swath_m=SW,pitch_m=P), " n=0 ->", v2(rect,0,swath_m=SW,pitch_m=P))
a=v2(rect,6,swath_m=SW,pitch_m=P); b=v2(rect,6,swath_m=SW,pitch_m=P)
print("確定性:", a==b)
if a!=b: fails.append("非確定性")

print("\n── 形狀掃描 ───────────────────────────────────────────────────")
L = Polygon([(-2000,-2000),(2000,-2000),(2000,0),(0,0),(0,2000),(-2000,2000)])
plus = Polygon([(-600,-2000),(600,-2000),(600,-600),(2000,-600),(2000,600),(600,600),
                (600,2000),(-600,2000),(-600,600),(-2000,600),(-2000,-600),(-600,-600)])
donut = Polygon([(-2000,-2000),(2000,-2000),(2000,2000),(-2000,2000)],
                [[(-700,-700),(700,-700),(700,700),(-700,700)]])
quad = [rect_xy(cx,0,625,2000) for cx in (-3750,-1250,1250,3750)]
scatter=[rect_xy(-4000,3000,500,500), rect_xy(3500,-2500,900,700), rect_xy(0,0,1500,400),
         rect_xy(-1000,-3500,300,1200), rect_xy(4200,3600,700,400)]
case("L 形", [L], 4); case("L 形", [L], 8)
case("十字形", [plus], 4); case("方環(帶洞)", [donut], 4)
case("斜置 37 度", [affinity.rotate(rect[0],37.0,origin=(0,0))], 8)
case("四塊", quad, 3); case("五塊散置", scatter, 6)

print("\n── 隨機凸包 ───────────────────────────────────────────────────")
rng = random.Random(11); ratios=[]
for t in range(6):
    pts=[(rng.uniform(-2600,2600), rng.uniform(-2200,2200)) for _ in range(7)]
    g=MultiPoint(pts).convex_hull
    if g.geom_type!="Polygon" or g.area < 4e6: continue
    n=rng.choice([3,4,6])
    m,m1,mb = case(f"隨機凸包#{t} {g.area/1e6:.1f}km2",[g],n)
    if m1["scan_rate"]>0: ratios.append(m["scan_rate"]/m1["scan_rate"])
print(f"\nv2/v1 掃描率比：min={min(ratios):.3f} 中位={sorted(ratios)[len(ratios)//2]:.3f} max={max(ratios):.3f}")
print()
print("!! 失敗:" if fails else "全部通過")
for f in fails: print("  -", f)
