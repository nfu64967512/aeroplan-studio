from pathlib import Path
"""消融實驗：algo_darp 自述的每一項「常識優化」，關掉它數字會不會真的變差？

判別稻草人的關鍵不是「程式碼裡有沒有寫」，而是「拿掉它答案會不會動」。
寫了卻沒生效的優化跟沒寫一樣（而且更難查）。
"""
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely.geometry import Polygon                           # noqa: E402

from bench import metrics, rect_xy                             # noqa: E402
import algo_darp as D                                          # noqa: E402

SWATH = 2 * 140.0 * math.tan(math.radians(60.0) / 2)
PITCH = SWATH * 0.7
RECT = [rect_xy(0, 0, 2500, 2000)]
L = [Polygon([(-2000, -2000), (2000, -2000), (2000, 0),
              (0, 0), (0, 2000), (-2000, 2000)])]
QUAD = [rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)]

CASES = (("5x4 矩形 n=8", RECT, 8), ("L 形 n=4", L, 4), ("四塊 n=8", QUAD, 8))
ORIG = {k: getattr(D, k) for k in ("_refine", "_darp", "_blocks")}
ORIG_ORDERS = D._Field._orders
ORIG_PATCH = D._Field._patch


def score(polys, n):
    tr = D.plan(polys, n, swath_m=SWATH, pitch_m=PITCH)
    m = metrics(polys, tr, SWATH)
    return m["max_km"], m["scan_rate"], m["coverage"]


def restore():
    for k, v in ORIG.items():
        setattr(D, k, v)
    D._Field._orders = staticmethod(ORIG_ORDERS)
    D._Field._patch = ORIG_PATCH


def ablate(name, apply_fn):
    restore()
    apply_fn()
    out = []
    for tag, polys, n in CASES:
        mx, sr, cv = score(polys, n)
        out.append(f"{tag}: max {mx:6.2f} km  掃描率 {sr:6.2f}  cov {cv * 100:6.2f}%")
    restore()
    print(f"{name:26s} | " + " | ".join(out))


def no_comb():
    D._Field._orders = staticmethod(lambda ks: [ks, ks[::-1]])


def no_refine():
    def stub(field, lab, m, start, budget=60):
        trks, lens = D._routes_of(field, lab, m, start)
        return lab, trks, lens
    D._refine = stub


def no_darp_loop():
    D._darp = lambda field, m, seeds, start: (None, float("inf"))


def only_b_eq_n():
    def stub(field, m, b):
        return ORIG["_blocks"](field, m, b) if b == m else None
    D._blocks = stub


def only_b_eq_1():
    def stub(field, m, b):
        return ORIG["_blocks"](field, m, b) if b == 1 else None
    D._blocks = stub


def no_patch():
    D._Field._patch = lambda self, rot, rows, swath, pitch, umin, umax: rows


if __name__ == "__main__":
    print("刈幅 %.1f m 間距 %.1f m，全部由 bench.metrics 評分\n" % (SWATH, PITCH))
    ablate("原版（全部優化開著）", lambda: None)
    ablate("關掉梳形順序", no_comb)
    ablate("關掉邊界局部搜尋 _refine", no_refine)
    ablate("關掉 DARP 修正係數迴路", no_darp_loop)
    ablate("只留 b=n（全寬橫帶）", only_b_eq_n)
    ablate("只留 b=1（單帶多欄）", only_b_eq_1)
    ablate("關掉凹形補洞 _patch", no_patch)
