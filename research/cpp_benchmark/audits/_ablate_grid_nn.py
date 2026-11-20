from pathlib import Path
"""grid_nn 消融：拿掉每一項常識優化各會退成什麼樣。跑三個情境。"""
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
B = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, B)

import numpy as np                                       # noqa: E402
from shapely.geometry import Polygon                     # noqa: E402
from bench import metrics, rect_xy                       # noqa: E402
import algo_grid_nn as A                                 # noqa: E402

SW = 2 * 140.0 * math.tan(math.radians(60.0) / 2)
PT = SW * 0.7
ST = (0.0, 0.0)
L = Polygon([(-1500, -1500), (1500, -1500), (1500, 0), (0, 0), (0, 1500), (-1500, 1500)])
SCEN = [("(1) rect 5x4 n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) four 1.25x4 n=8",
         [rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)], 8),
        ("(3) L-shape n=4", [L], 4)]


def build(polys, n, *, grid_align=True, prune=True, policies=(True, False),
          rebal=True, local="2opt+oropt"):
    cell = A._cell_size(PT, SW)
    old = A._min_width_angle
    if not grid_align:
        A._min_width_angle = lambda p: 0.0
    c, cl, fr = A._build_cells(polys, cell)
    A._min_width_angle = old
    if prune:
        k = A._prune(c, cl, fr, SW, cell)
        c = [c[i] for i in k]
    P = np.asarray(c, float)
    best = None
    for dc in policies:
        groups = A._cluster(P, n, ST, cell, depot_caps=dc)
        if rebal:
            groups = A._rebalance(groups, c, ST, cell)
        tr, mk = [], 0.0
        for g in groups:
            if not g:
                continue
            pts = A._dedupe([ST] + [c[i] for i in g])
            m = len(pts)
            if m < 2:
                continue
            D, neigh = A._mat(np.asarray(pts, float))
            tour = A._nn_tour(D, m)
            pos = [0] * m
            for t, x in enumerate(tour):
                pos[x] = t
            if local != "nn":
                for _ in range(30):
                    a = A._two_opt(tour, D, neigh, pos) if "2opt" in local else False
                    b = A._or_opt(tour, D, neigh, pos) if "oropt" in local else False
                    if not (a or b):
                        break
            kk = tour.index(0)
            seq = tour[kk:] + tour[:kk]
            tr.append([ST] + [pts[i] for i in seq[1:]] + [ST])
            mk = max(mk, A._tour_len(tour, D))
        if tr and (best is None or mk < best[0] - 1e-9):
            best = (mk, tr)
    return best[1] if best else []


VAR = [
    ("完整版（兩種容量政策取優）", {}),
    ("NN only（無 2-opt/Or-opt）", dict(local="nn")),
    ("NN + 2-opt（無 Or-opt）", dict(local="2opt")),
    ("只用等量容量（無重啟）", dict(policies=(False,))),
    ("只用 home 補償容量（無重啟）", dict(policies=(True,))),
    ("網格不對齊最小寬度", dict(grid_align=False)),
    ("不做碎格剪枝", dict(prune=False)),
    ("不做 makespan 再平衡", dict(rebal=False)),
]

for title, polys, n in SCEN:
    print(f"\n### {title}")
    print(f"{'變體':30s}{'總 km':>8s}{'最長 km':>9s}{'覆蓋':>8s}{'開銷':>7s}"
          f"{'不均':>7s}{'掃描率':>9s}")
    for name, kw in VAR:
        tr = build(polys, n, **kw)
        m = metrics(polys, tr, SW)
        print(f"{name:30s}{m['total_km']:8.1f}{m['max_km']:9.2f}"
              f"{m['coverage'] * 100:7.1f}%{m['overhead'] or 0:7.3f}"
              f"{m['balance'] or 0:7.3f}{m['scan_rate']:9.2f}")
