# -*- coding: utf-8 -*-
"""量 .ac 的軸向,含離群過濾(FG 的機體檔常帶少量遠處標記點)。"""
import math
import sys

path = sys.argv[1]
V = []
with open(path, encoding="utf-8", errors="replace") as fh:
    it = iter(fh)
    for ln in it:
        if ln.startswith("numvert"):
            n = int(ln.split()[1])
            for _ in range(n):
                p = next(it).split()
                if len(p) >= 3:
                    V.append((float(p[0]), float(p[1]), float(p[2])))
print("vertices: %d" % len(V))

# 用 1/99 百分位當界,丟掉離群
def pct(vals, q):
    s = sorted(vals)
    return s[max(0, min(len(s) - 1, int(q * (len(s) - 1))))]

bnds = []
for i in range(3):
    col = [v[i] for v in V]
    bnds.append((pct(col, 0.005), pct(col, 0.995)))
V = [v for v in V if all(bnds[i][0] <= v[i] <= bnds[i][1] for i in range(3))]
print("after outlier filter: %d" % len(V))

names = "XYZ"
lo = [min(v[i] for v in V) for i in range(3)]
hi = [max(v[i] for v in V) for i in range(3)]
ln_ = [hi[i] - lo[i] for i in range(3)]
for i in range(3):
    print("  %s: [%8.3f %8.3f]  len %7.3f" % (names[i], lo[i], hi[i], ln_[i]))

axis = max(range(3), key=lambda i: ln_[i])
others = [i for i in range(3) if i != axis]
span = max(others, key=lambda i: ln_[i])
up = [i for i in others if i != span][0]
print("  長軸=%s  翼展=%s  上下=%s" % (names[axis], names[span], names[up]))

N = 14
step = ln_[axis] / N
prof = []
for k in range(N):
    a0 = lo[axis] + k * step
    r = [math.hypot(v[others[0]], v[others[1]])
         for v in V if a0 <= v[axis] < a0 + step]
    if len(r) < 30:
        prof.append((a0, len(r), float("nan"))); continue
    r.sort()
    prof.append((a0, len(r), r[int(0.95 * (len(r) - 1))]))
for a0, n, r in prof:
    print("   %8.3f %7d %8.3f" % (a0, n, r))
f = next(p for p in prof if p[1] >= 30)
l = next(p for p in reversed(prof) if p[1] >= 30)
print("  低端 r95 %.3f / 高端 r95 %.3f" % (f[2], l[2]))
print("  => 收尖端(機鼻)在 %s 的 %s 方向"
      % (names[axis], "正" if l[2] < f[2] else "負"))
