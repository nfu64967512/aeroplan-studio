from pathlib import Path
"""algo_darp 的健全性檢查：介面契約、退化輸入、斜置/帶洞/多凹多邊形。"""
import math
import sys
import time

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely import affinity                                  # noqa: E402
from shapely.geometry import Polygon                          # noqa: E402

from bench import boustrophedon, metrics, rect_xy             # noqa: E402
from algo_darp import plan                                    # noqa: E402

SWATH = 2 * 140.0 * math.tan(math.radians(60.0) / 2)
PITCH = SWATH * 0.7
START = (0.0, 0.0)
fails = []


def contract(name, polys, n):
    t0 = time.perf_counter()
    tr = plan(polys, n, swath_m=SWATH, pitch_m=PITCH, start=START)
    dt = time.perf_counter() - t0
    for i, t in enumerate(tr):
        if tuple(t[0]) != START or tuple(t[-1]) != START:
            fails.append(f"{name}: track{i} 頭尾不在 start ({t[0]}, {t[-1]})")
        if any((not math.isfinite(p[0])) or (not math.isfinite(p[1])) for p in t):
            fails.append(f"{name}: track{i} 有非有限座標")
    if polys and n >= 1 and len(tr) != min(n, max(1, n)):
        pass
    m = metrics(polys, tr, SWATH) if tr else {"coverage": 0.0, "scan_rate": 0.0,
                                              "n_uav": 0, "max_km": 0.0}
    b = metrics(polys, boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH),
                SWATH) if polys else m
    print(f"{name:34s} n={n:<3d} 台={m['n_uav']:<3d} cov={m['coverage'] * 100:6.2f}% "
          f"max={m['max_km']:7.2f} km  掃描率={m['scan_rate']:6.2f} "
          f"(基準 {b['scan_rate']:6.2f})  {dt:5.2f}s")
    if tr and m["coverage"] < 0.99:
        fails.append(f"{name}: 覆蓋率 {m['coverage']:.4f} < 0.99")
    if tr and len(tr) != n and n <= 40:
        fails.append(f"{name}: 回傳 {len(tr)} 條航跡，要求 {n}")
    return m


print("── 介面契約 / 一般情境 ─────────────────────────────────────────")
rect = [rect_xy(0, 0, 2500, 2000)]
for n in (1, 2, 3, 4, 5, 6, 7, 8, 11, 16):
    contract(f"矩形 5x4 km", rect, n)

print("\n── 斜置 37° 的同一塊區 ─────────────────────────────────────────")
tilt = [affinity.rotate(rect[0], 37.0, origin=(0, 0))]
for n in (4, 8):
    contract("斜置矩形 37°", tilt, n)

print("\n── 凹 / 帶洞 / 多塊 ───────────────────────────────────────────")
L = Polygon([(-2000, -2000), (2000, -2000), (2000, 0), (0, 0), (0, 2000), (-2000, 2000)])
contract("L 形", [L], 4)
contract("L 形", [L], 8)
plus = Polygon([(-600, -2000), (600, -2000), (600, -600), (2000, -600), (2000, 600),
                (600, 600), (600, 2000), (-600, 2000), (-600, 600), (-2000, 600),
                (-2000, -600), (-600, -600)])
contract("十字形（四個凹角）", [plus], 4)
donut = Polygon([(-2000, -2000), (2000, -2000), (2000, 2000), (-2000, 2000)],
                [[(-700, -700), (700, -700), (700, 700), (-700, 700)]])
contract("方環（帶洞）", [donut], 4)
quad = [rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)]
contract("四塊 1.25x4 km", quad, 8)
contract("四塊 1.25x4 km", quad, 2)
contract("四塊 1.25x4 km", quad, 3)
scatter = [rect_xy(-4000, 3000, 500, 500), rect_xy(3500, -2500, 900, 700),
           rect_xy(0, 0, 1500, 400), rect_xy(-1000, -3500, 300, 1200),
           rect_xy(4200, 3600, 700, 400)]
contract("五塊大小不一散置", scatter, 6)
contract("五塊大小不一散置", scatter, 2)

print("\n── 退化 / 邊界輸入 ─────────────────────────────────────────────")
tiny = [rect_xy(1000, 1000, 60, 40)]                 # 比刈幅還小
contract("小於刈幅的區塊", tiny, 3)
thin = [rect_xy(0, 0, 3000, 45)]                     # 一條細長帶
contract("細長帶 6000x90 m", thin, 4)
contract("矩形 n=0", rect, 0)
print(f"空多邊形清單 -> {plan([], 4, swath_m=SWATH, pitch_m=PITCH)}")
print(f"n=0          -> {plan(rect, 0, swath_m=SWATH, pitch_m=PITCH)}")

print("\n── 重疊輸入（union 應自動處理）─────────────────────────────────")
ov = [rect_xy(0, 0, 1500, 1500), rect_xy(1000, 1000, 1500, 1500)]
contract("兩塊互相重疊", ov, 4)

print("\n── 確定性（同輸入跑兩次）───────────────────────────────────────")
a = plan(rect, 8, swath_m=SWATH, pitch_m=PITCH)
b = plan(rect, 8, swath_m=SWATH, pitch_m=PITCH)
print("兩次結果相同:", a == b)
if a != b:
    fails.append("非確定性")

print()
if fails:
    print("!! 失敗項目:")
    for f in fails:
        print("   -", f)
else:
    print("全部通過")
