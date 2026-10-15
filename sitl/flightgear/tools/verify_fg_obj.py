# -*- coding: utf-8 -*-
"""驗證匯出的 OBJ 真的在 FG 方向:機鼻 -X、上 +Y、翼展 ±Z。
⚠️ 不看畫面就下結論的唯一辦法是量。這裡量三件事:
   1. 哪一軸最長(應是 Z = 翼展 2.325)
   2. 機身軸(X)上,哪一端收尖(應是 -X)
   3. 主翼在 X 的哪裡(展寬最大處)
"""
import math
import sys

V = []
for ln in open(sys.argv[1], encoding="utf-8", errors="replace"):
    if ln.startswith("v "):
        p = ln.split()
        V.append((float(p[1]), float(p[2]), float(p[3])))
print("vertices: %d" % len(V))
names = "XYZ"
lo = [min(v[i] for v in V) for i in range(3)]
hi = [max(v[i] for v in V) for i in range(3)]
for i in range(3):
    print("  %s: [%8.4f %8.4f] len %7.4f" % (names[i], lo[i], hi[i], hi[i]-lo[i]))

exp = {"X": 3.786, "Y": 0.901, "Z": 2.325}
ok = True
for i, n in enumerate(names):
    got = hi[i] - lo[i]
    good = abs(got - exp[n]) < 0.02
    print("  %s 長度 %.4f  期望 %.3f  %s" % (n, got, exp[n], "OK" if good else "✗"))
    ok = ok and good

N = 12
step = (hi[0] - lo[0]) / N
print("\n  %8s %7s %9s %9s" % ("X from", "n", "r95(YZ)", "Z展寬"))
prof = []
for k in range(N):
    a0 = lo[0] + k * step
    sel = [v for v in V if a0 <= v[0] < a0 + step]
    if len(sel) < 20:
        print("  %8.3f %7d       --        --" % (a0, len(sel))); continue
    r = sorted(math.hypot(v[1], v[2]) for v in sel)
    zr = max(v[2] for v in sel) - min(v[2] for v in sel)
    prof.append((a0, r[int(0.95*(len(r)-1))], zr))
    print("  %8.3f %7d %9.4f %9.4f" % (a0, len(sel), prof[-1][1], zr))

f, l = prof[0], prof[-1]
nose_neg = f[1] < l[1]
print("\n  低 X 端 r95 %.4f / 高 X 端 r95 %.4f" % (f[1], l[1]))
print("  => 機鼻在 X 的 %s 方向 %s" % ("負" if nose_neg else "正",
                                      "OK(FG 要的)" if nose_neg else "✗ 方向反了"))
wing = max(prof, key=lambda p: p[2])
print("  主翼(展寬最大)在 X ≈ %.2f,展寬 %.3f" % (wing[0], wing[2]))
sys.exit(0 if (ok and nose_neg) else 1)
