#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test I: is 0.063 robust to every timebase / differencing choice available?"""
from __future__ import annotations
import math, statistics as st
import numpy as np
import d1_replicate as R


def gain_corr(x, y):
    g = float(x @ y / max(x @ x, 1e-12))
    xc, yc = x - x.mean(), y - y.mean()
    return g, float(xc @ yc / max(math.sqrt((xc@xc)*(yc@yc)), 1e-12))


def central(t, y, h):
    n = len(t); s = np.full(n, np.nan)
    for k in range(h, n - h):
        dt = t[k + h] - t[k - h]
        if 0.05 <= dt <= 0.6:
            s[k] = (y[k + h] - y[k - h]) / dt
    return s


VAR = {}
for tag in R.TAGS:
    d = R.load(tag)
    if len(d["t"]) < 60: continue
    t, gam = d["t"], d["gam"]
    gc, _, _ = R.replay_gamcmd(t, d["r"], d["z"], gam, d["gs"])
    # timebase variants
    idx_t = np.arange(len(t)) * 0.05                      # pure row-index clock
    raw = d["t_raw"]
    gapboundary = np.zeros(len(t), bool)
    dt1 = np.diff(t, prepend=t[0]); dt2 = np.diff(t, append=t[-1])
    big = (dt1 > 0.6) | (dt2 > 0.6)
    for k in np.where(big)[0]:
        gapboundary[max(0, k - 12):k + 13] = True

    cases = {}
    _, iu = np.unique(t, return_index=True); iu = np.sort(iu)
    y0, _ = R.local_slope(t[iu], gam[iu], 0.5)
    cases["gate: t_dq, unique(), local_slope +-0.5s"] = (
        (gc[iu] - gam[iu]) / 1.0, y0)
    y1, _ = R.local_slope(idx_t, gam, 0.5)
    cases["row-index clock, NO rows dropped"] = ((gc - gam) / 1.0, y1)
    m = ~gapboundary
    _, iu2 = np.unique(t[m], return_index=True); iu2 = np.sort(iu2)
    tt = t[m][iu2]
    y2, _ = R.local_slope(tt, gam[m][iu2], 0.5)
    cases["freeze-boundary ticks removed (+-0.6 s)"] = (
        (gc[m][iu2] - gam[m][iu2]) / 1.0, y2)
    for h, lab in [(1, "central +-1 row (0.1 s)"), (2, "central +-2 rows (0.2 s)"),
                   (5, "central +-5 rows (0.5 s)"), (10, "central +-10 rows (1.0 s)"),
                   (20, "central +-20 rows (2.0 s)")]:
        yh = central(t, gam, h)
        mm = np.isfinite(yh)
        cases[lab] = ((gc[mm] - gam[mm]) / 1.0, yh[mm])
    for k, (x, y) in cases.items():
        g, c = gain_corr(np.asarray(x), np.asarray(y))
        VAR.setdefault(k, []).append((g, c))

print("=== the 'achieved/demanded gamma-rate gain', under every timebase and")
print("    differencing choice the CSV admits (median over 32 sorties) ===")
print(f"{'variant':<46s}{'gain':>9s}{'corr':>9s}")
for k, v in VAR.items():
    g = [a for a, _ in v]; c = [b for _, b in v]
    print(f"{k:<46s}{st.median(g):>9.4f}{st.median(c):>+9.4f}")
