# -*- coding: utf-8 -*-
from __future__ import annotations
import math, random, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
G = 9.80665
acs = F.load()
random.seed(20260822)


def perm_p(x, y, n=200000):
    """two-sided permutation p on difference of medians."""
    obs = abs(st.median(x) - st.median(y))
    pool = list(x) + list(y)
    k = len(x)
    hit = 0
    for _ in range(n):
        random.shuffle(pool)
        if abs(st.median(pool[:k]) - st.median(pool[k:])) >= obs - 1e-12:
            hit += 1
    return (hit + 1) / (n + 1)


print("=== Q1c  rate-limited slew: what clampON can actually deliver in the band")
print("  clampON re-clamps the post-floor command back into [-1,3] g, so the")
print("  command is a RATE limit  gdot = (nz_min - cos gam)*g/V , not an angle.")
print("  authority(meas) = p05 of the 1 s achieved nose-down rate while PN was")
print("  genuinely in command (post-handover): see Q3d.")
AUTH = -5.45
h = "%-9s %s %8s %8s %8s %8s %8s %8s %8s"
print(h % ("tag", "F", "band_s", "gam_ent", "rate_cmd", "t_to_0", "gam_los",
           "t_to_LOS", "verdict"))
for a in acs:
    b = a["band_clampON"]
    ge = D(b[0]["gam"])
    rate = st.median([D((-1.0 - math.cos(r["gam"])) * G / r["V"]) for r in b])
    bs = b[-1]["t"] - b[0]["t"]
    los = D(b[-1]["gam_los"])
    t0 = (ge / abs(rate)) if ge > 0 else 0.0
    tl = (ge - los) / abs(rate)
    v = "reaches LOS" if tl <= bs else ("levels only" if t0 <= bs else "still climbing")
    print(h % (a["tag"], "*" if a["tag"] in B.FAIL else " ", "%.1f" % bs,
               "%+.2f" % ge, "%.2f" % rate, "%.1f" % t0, "%.1f" % los,
               "%.1f" % tl, v))

print("\n=== statistics: 6 failures vs 26 successes, band quantities")
print("  n=32 airframes.  MULTIPLE-COMPARISON EXPOSURE: 6 pre-specified band")
print("  quantities tested below; no correction applied, so read p*6 as a crude")
print("  Bonferroni bound.  This campaign has already produced several 15/16")
print("  in-sample separations, so none of these is decisive on its own.")
def dh(a): return a["band_clampOFF"][-1]["h"] - a["band_clampOFF"][0]["h"]
QTY = [
    ("dh_band_m", dh),
    ("gam_max_deg", lambda a: max(D(r["gam"]) for r in a["band_clampOFF"])),
    ("gam_med_deg", lambda a: st.median([D(r["gam"]) for r in a["band_clampOFF"]])),
    ("los_at_handover_deg", lambda a: -D(a["band_clampOFF"][-1]["gam_los"])),
    ("V_at_handover", lambda a: a["band_clampOFF"][-1]["V"]),
    ("req_nz_min_band", lambda a: min(r["req_nz"] for r in a["band_clampOFF"])),
]
print("  %-22s %10s %10s %9s" % ("quantity", "fail med", "ok med", "perm p"))
for nm, f in QTY:
    x = [f(a) for a in acs if a["tag"] in B.FAIL]
    y = [f(a) for a in acs if a["tag"] not in B.FAIL]
    print("  %-22s %10.2f %10.2f %9.4f" % (nm, st.median(x), st.median(y), perm_p(x, y, 50000)))

print("\n=== throttle shape: is the PN throttle bang-bang in the band?")
thr = [r["thr"] for a in acs for r in a["band_clampOFF"]]
mid = [x for x in thr if 1e-9 < x < 1.0 - 1e-9]
print("  n=%d  thr==0 %.2f%%  thr==1 %.2f%%  strictly between %.2f%%"
      % (len(thr), 100 * sum(1 for x in thr if x <= 1e-9) / len(thr),
         100 * sum(1 for x in thr if x >= 1 - 1e-9) / len(thr),
         100 * len(mid) / len(thr)))
