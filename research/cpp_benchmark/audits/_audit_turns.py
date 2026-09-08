"""Confound audit: the ruler is pure geometry, so a method that ignores turn radius
gets a free discount. Count how sharp each method's corners actually are.

A 200 m-radius fixed-wing cannot execute a corner at all; it must fly an arc whose
extra length grows with the turn angle. So a method with thousands of >90 deg corners
is being scored on a route it could not fly, while a method that already inserted the
turning geometry is charged for it.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix import METHODS, SCEN                             # noqa: E402


def profile(tracks):
    """turn-angle histogram + total length"""
    bins = {"<30": 0, "30-60": 0, "60-90": 0, ">90": 0}
    tot = 0.0
    for t in tracks:
        for i in range(1, len(t)):
            tot += math.dist(t[i], t[i - 1])
        for i in range(1, len(t) - 1):
            ax, ay = t[i][0] - t[i - 1][0], t[i][1] - t[i - 1][1]
            bx, by = t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1]
            na, nb = math.hypot(ax, ay), math.hypot(bx, by)
            if na < 1e-9 or nb < 1e-9:
                continue
            c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
            d = math.degrees(math.acos(c))
            bins["<30" if d < 30 else "30-60" if d < 60 else "60-90" if d < 90
                 else ">90"] += 1
    return bins, tot


def dubins_penalty(tracks, R=200.0):
    """Lower bound on the extra distance a radius-R vehicle must fly.

    At a corner of exterior angle d the vehicle must fly an arc instead of the corner.
    Cutting the corner is not allowed here (it would leave the strip), so charge the
    classic 'fly past, turn, come back' bound: extra >= R*(tan(d/2)*2 - d_rad*R/R)
    -> extra = 2*R*tan(d/2) - R*d_rad, the arc-vs-corner difference. Capped at d<=170
    deg because a true reversal needs a full teardrop, handled separately.
    """
    extra, hairpins = 0.0, 0
    for t in tracks:
        for i in range(1, len(t) - 1):
            ax, ay = t[i][0] - t[i - 1][0], t[i][1] - t[i - 1][1]
            bx, by = t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1]
            na, nb = math.hypot(ax, ay), math.hypot(bx, by)
            if na < 1e-9 or nb < 1e-9:
                continue
            c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
            d = math.acos(c)
            if math.degrees(d) > 170.0:
                hairpins += 1
                extra += math.pi * R          # teardrop reversal, crude
                continue
            extra += 2.0 * R * math.tan(d / 2.0) - R * d
    return extra, hairpins


if __name__ == "__main__":
    scen, n = sys.argv[1] if len(sys.argv) > 1 else "S1", int(sys.argv[2]) if len(sys.argv) > 2 else 8
    polys, _ = SCEN[scen]
    print(f"=== {scen} n={n} : corner profile and radius-200 m penalty ===")
    print(f"{'method':18s}{'len km':>8s}{'<30':>7s}{'30-60':>7s}{'60-90':>7s}{'>90':>7s}"
          f"{'hairpin':>8s}{'+R200 km':>10s}{'+%':>7s}")
    for m in METHODS:
        try:
            tracks, _ms = METHODS[m][2](polys, n)
        except Exception as e:                                # noqa: BLE001
            print(f"{m:18s} ERROR {type(e).__name__}")
            continue
        if not tracks:
            print(f"{m:18s} EMPTY")
            continue
        b, tot = profile(tracks)
        ex, hp = dubins_penalty(tracks)
        print(f"{m:18s}{tot / 1000:8.1f}{b['<30']:7d}{b['30-60']:7d}{b['60-90']:7d}"
              f"{b['>90']:7d}{hp:8d}{ex / 1000:10.1f}{ex / tot * 100:6.1f}%")
