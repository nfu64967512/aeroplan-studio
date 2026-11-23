from pathlib import Path
# 探針 4：(A) 細粒度精修 與 (B) V 形順序 各自貢獻多少？S2 的跨區切割還有多少肉？
import math, sys, time
import numpy as np
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely.geometry import Polygon
from bench import rect_xy, metrics
import algo_darp as D
import algo_darp_v2 as V

SW = 2*140.0*math.tan(math.radians(60.0)/2); P = SW*0.7
S = (0.0, 0.0)
L = Polygon([(-2000,-2000),(2000,-2000),(2000,0),(0,0),(0,2000),(-2000,2000)])
CASES = [("S1 rect n=8", [rect_xy(0,0,2500,2000)], 8), ("S3 L n=4", [L], 4)]

def run(polys, n, fine, vord):
    of, opf = D._Field, D._plan_field
    class F(D._Field):
        @staticmethod
        def _orders(ks):
            return V._orders_v2(ks) if vord else V._ORIG_ORDERS(ks)
    def pf(field, m, start):
        return V._plan_field_v2(field, m, start) if fine else opf(field, m, start)
    D._Field, D._plan_field = F, pf
    try:
        t0=time.perf_counter(); tr = D.plan(polys, n, swath_m=SW, pitch_m=P, start=S)
        return metrics(polys, tr, SW), time.perf_counter()-t0
    finally:
        D._Field, D._plan_field = of, opf

print(f"{'case':12s}{'A細精修':>9s}{'B V順序':>9s}{'max km':>9s}{'total':>9s}{'scan':>8s}{'sec':>7s}")
for name, polys, n in CASES:
    for fine, vord in ((0,0),(1,0),(0,1),(1,1)):
        m, dt = run(polys, n, fine, vord)
        print(f"{name:12s}{'v' if fine else '-':>9s}{'v' if vord else '-':>9s}"
              f"{m['max_km']:9.2f}{m['total_km']:9.1f}{m['scan_rate']:8.2f}{dt:7.1f}")

print("\n── S2 跨區切割能不能吃到分數解？ ──")
quad = [rect_xy(cx,0,625,2000) for cx in (-3750,-1250,1250,3750)]
fields = [D._Field(p, SW, P) for p in quad]
print("每塊條帶數:", [f.K for f in fields], " work km:", [round(f.work/1000,1) for f in fields])
alt = D._plan_chunks(fields, 8, S)
print("v1 _plan_chunks (等權重切) max=%.2f 總=%.1f  台=%d" %
      (max(D._plen(t) for t in alt)/1000, sum(D._plen(t) for t in alt)/1000, len(alt)))
m = metrics(quad, alt, SW); print("  metrics:", {k:m[k] for k in ('max_km','coverage','scan_rate')})
# 直接對切點做局部搜尋（用實測長度）
order = D._geo_order(fields)
seq = [(fi, ci) for fi in order for ci in D._cell_order(fields[fi])]
N = len(seq)
def build(cuts):
    out, prev = [], 0
    for c in list(cuts)+[N]:
        ch = seq[prev:c]; prev = c
        by = {}
        for fi, ci in ch: by.setdefault(fi, []).append(ci)
        t = D._chain(fields, order, by, S)
        out.append(t)
    return out
cuts = [round(N*(i+1)/8) for i in range(7)]
def key(cuts):
    ts = build(cuts); ls = [D._plen(t) for t in ts]
    if len(ts)!=8 or min(ls)<=0: return (1e18,1e18)
    return (round(max(ls),3), round(sum(ls),3))
cur = key(cuts)
for it in range(4000):
    best=None
    for i in range(7):
        for d in (-16,-8,-4,-2,-1,1,2,4,8,16):
            c2 = list(cuts); c2[i]+=d
            if not (0 < c2[i] < N) or c2 != sorted(c2) or len(set(c2))<7: continue
            k2 = key(c2)
            if k2 < cur and (best is None or k2 < best[0]): best=(k2,c2)
    if best is None: break
    cur, cuts = best
print(f"切點局部搜尋後: max={cur[0]/1000:.2f} 總={cur[1]/1000:.1f}")
m = metrics(quad, build(cuts), SW); print("  metrics:", {k:m[k] for k in ('max_km','coverage','scan_rate','balance')})
