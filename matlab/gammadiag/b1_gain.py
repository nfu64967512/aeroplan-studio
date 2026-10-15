# -*- coding: utf-8 -*-
"""Small-signal tracking gain in the regime clampON would actually operate in."""
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
acs = F.load()


def pairs(sel, lo, hi):
    out = []
    for a in acs:
        if not sel(a):
            continue
        seq = a["pn_flown"]
        for i in range(len(seq)):
            j = i
            while j < len(seq) - 1 and seq[j]["t"] - seq[i]["t"] < 1.0:
                j += 1
            dt = seq[j]["t"] - seq[i]["t"]
            if not (0.8 <= dt <= 1.3) or seq[i]["d"] < 300.0:
                continue
            cmd = D(seq[i]["gam_cmd"] - seq[i]["gam"])
            if not (lo <= abs(cmd) <= hi):
                continue
            out.append((cmd, D(seq[j]["gam"] - seq[i]["gam"]) / dt, a["tag"]))
    return out


def fit(p):
    if len(p) < 30:
        return None
    x = [a for a, b, c in p]; y = [b for a, b, c in p]
    mx, my = st.mean(x), st.mean(y)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    sxx = sum((a - mx) ** 2 for a in x)
    syy = sum((b - my) ** 2 for b in y)
    return len(p), sxy / sxx, sxy / math.sqrt(sxx * syy), st.median(x), st.median(y)


print("post-handover, d>=300 m, 1 s achieved window.  cmd = gam_cmd - gam (deg,")
print("== deg/s at pn_lead=1).  slope = achieved rate / commanded rate.")
h = "%-28s %7s %8s %8s %9s %9s"
print(h % ("population / cmd band", "n", "slope", "pearson", "cmd med", "ach med"))
for nm, sel in (("ALL 32", lambda a: True),
                ("NZC (clamp_after_floor=1)", lambda a: a["run"].startswith("NZC")),
                ("RRE (clamp_after_floor=0)", lambda a: a["run"].startswith("RRE"))):
    for lo, hi, lbl in ((0.5, 1e9, "any"), (0.5, 8.0, "|cmd|<=8"),
                        (3.0, 8.0, "3..8"), (8.0, 1e9, ">8")):
        r = fit(pairs(sel, lo, hi))
        if r:
            print(h % ("%s  %s" % (nm, lbl), r[0], "%.4f" % r[1], "%.4f" % r[2],
                       "%+.2f" % r[3], "%+.2f" % r[4]))
