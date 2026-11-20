"""量「圍籬方向沒跟掃描角一致」到底浪費多少面積。

用同一組掃描航線，比較三種矩形：
  aabb  目前行為（軸對齊包絡）—— dual_map_widget.py:592 沒傳 method，預設就是它
  mrr   最小旋轉矩形（程式裡已經有，只差一個參數）
  scan  真正對齊掃描角的矩形（目前**不存在**，要新增）

    python scripts/_fence_angle_probe.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from shapely.geometry import MultiPoint, Polygon
from shapely import affinity


def scan_lines(angle_deg: float, n: int = 8, length: float = 600.0,
               spacing: float = 60.0):
    """在原點附近產生 n 條間距 spacing 的平行掃描線（含折返點）。"""
    a = math.radians(angle_deg)
    ux, uy = math.cos(a), math.sin(a)          # 沿線方向
    vx, vy = -math.sin(a), math.cos(a)         # 線間方向
    pts = []
    for i in range(n):
        off = (i - (n - 1) / 2.0) * spacing
        cx, cy = vx * off, vy * off
        for t in (-length / 2, length / 2):
            pts.append((cx + ux * t, cy + uy * t))
    return pts


def rect_aligned(pts, angle_deg: float) -> Polygon:
    """把點集旋到掃描角的框架裡取軸對齊包絡，再轉回來。"""
    mp = MultiPoint(pts)
    rot = affinity.rotate(mp, -angle_deg, origin=(0, 0))
    return affinity.rotate(rot.envelope, angle_deg, origin=(0, 0))


print("掃描角  aabb 面積   mrr 面積   scan 面積   aabb/scan   mrr 是否==scan")
print("-" * 72)
worst = (0.0, 0.0)
for ang in (0, 10, 20, 30, 40, 45, 50, 60, 75, 90):
    pts = scan_lines(ang)
    mp = MultiPoint(pts)
    a_aabb = mp.envelope.area
    a_mrr = mp.minimum_rotated_rectangle.area
    a_scan = rect_aligned(pts, ang).area
    ratio = a_aabb / a_scan
    same = "是" if abs(a_mrr - a_scan) / a_scan < 0.01 else "**否**"
    print("%5d°  %9.0f  %9.0f  %9.0f  %8.2fx   %s"
          % (ang, a_aabb, a_mrr, a_scan, ratio, same))
    if ratio > worst[1]:
        worst = (ang, ratio)

print()
print("最糟：掃描角 %d° 時，目前的 aabb 圍籬面積是必要面積的 %.2f 倍"
      % (worst[0], worst[1]))

# ── mrr 會不會跟掃描角不一致？──────────────────────────────────────
# 加上「子區域之間的轉場航線」——真實 DCCPP 結果一定有這些點，
# 它們會把點集的最小旋轉矩形拉離掃描角。
print()
print("加入轉場點之後（模擬真實 DCCPP：掃描線 + 子區之間的轉場）：")
pts = scan_lines(30.0)
pts += [(-500.0, 420.0), (520.0, -430.0)]      # 兩個離群的轉場點
mp = MultiPoint(pts)
a_mrr = mp.minimum_rotated_rectangle.area
a_scan = rect_aligned(pts, 30.0).area
print("  mrr 面積 %.0f  vs  對齊掃描角 %.0f  ->  差 %.1f%%"
      % (a_mrr, a_scan, abs(a_mrr - a_scan) / a_scan * 100))
print("  ⇒ mrr 是「最小面積」，不是「對齊掃描角」。兩者不是同一件事。")
