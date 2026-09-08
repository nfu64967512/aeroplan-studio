"""grid_nn_v2b —— grid_nn 的改進版（審查者第 2 位）。改的是**必訪點集的幾何**。

⚠ 檔名：`algo_grid_nn_v2.py` 已被第 1 位審查者佔用（他改視點內縮 / 再平衡 /
合併共線點），這一支改的是**列的擺法與取樣**，兩邊互不重疊，所以另開檔名。

一、v1 沒有被寫爛（先講清楚，免得這份改進被讀成「原版很爛」）
--------------------------------------------------------------
* 求解器幾乎收斂：場內航程 / (格數 x 格距) = **1.019**（rect n=8）、1.002（L）、
  1.034（三角形）。2-opt/Or-opt 真的在做事。
* 分配到位：balance 1.003~1.008，8 台全用上。
* 覆蓋率是 **1.000000** 的原值，不是四捨五入。
* 掃描方向確實取最小寬度（四個形狀對過 `bench.min_width_angle`，逐位元相同）。
* 它自陳「剪枝在 pitch=0.7 一格都沒剪到」也屬實（rect/L/tri 實測 pruned=0）。

二、v1 真正的損失在**必訪點集怎麼鋪**，有三筆，可分別量
--------------------------------------------------------
A. **多一列**。點式網格把列數綁在「用 cell 鋪滿 bbox」= ceil(H/cell) = 36 列。
   但覆蓋只需要 ceil((H − swath)/pitch) + 1 = **35 列**：首末列離邊界 swath/2
   就夠，因為邊界剛好落在刈幅圓上。5.0x4.0 km 少一列 = 少飛 4.98 km。
   v2b 的做法是「**能達到最少列數的最小內縮**」：
       k = ceil((H − 2r)/pitch);  e = max(0, (H − k·pitch)/2);  s = (H − 2e)/k
   e 取最小是為了讓外側列還能用比較粗的取樣（見 C）。
B. **端點沒有修剪**。格網置中後每列兩端只內縮 10.5 m；幾何允許的是
   T = sqrt(r² − (s/2)²) = 57.3 m（內側列；最壞點是「邊界 x 兩列中間」，
   距離剛好 sqrt(T² + (s/2)²) = r）。首末列不能修（外側角落沒人接）。
C. **取樣不能靠「巡迴會把整列走成一條線」**。我第一版讓外側列直接內縮 r 再照
   常取樣，覆蓋率就掉到 **99.99%**（外側列的取樣間隙 + 群間接縫會漏）。
   點式模型的覆蓋保證必須**光靠各點的刈幅圓**成立，於是三條不等式互相牽制：
       sqrt((d_out/2)² + e²)     ≤ r     外側列
       sqrt((d_in /2)² + (s/2)²) ≤ r     內側列
       sqrt(T²        + (s/2)²)  ≤ r     內側列端點修剪
   rect 5x4 km 的解：e = 77.0、d_out = 45.6、s = 113.16、d_in = 113.1、T = 56.2。
   外側兩列取樣加密只多約 200 點，而「走完一列的長度」跟取樣密度無關，所以不花錢。

三、還有一筆是**格點對齊**，不是幾何而是求解器賴以生存的性質
------------------------------------------------------------
取樣點必須落在**全域 u 格點**上。每列各自 `L/ceil(L/d)` 均分看起來一樣，但相鄰
兩列的點會在 u 上錯開 ⇒ 換列從付 s 變成付 sqrt(s² + du²)，v1「走鋸齒邊界不用錢」
的性質就沒了。實測差 **6.2 km**（193.92 vs 200.11，情境(1)）。
這一條是我一開始整支改寫成線段模型會慘敗（216.6 km，+10%）的同一個原因。

四、求解器**原封不動沿用 v1 的**
--------------------------------
`_v1._cluster` / `_v1._rebalance` / `_v1._solve` 直接 import 進來用。這樣改善幅度
100% 可歸因到點集幾何，不會跟求解器調校混在一起。兩種容量政策一樣都跑、取優。

五、實測（bench.metrics，`_rv2_run.py` / `_rv2_ablate.py` / `_rv2_sweep.py` 可重跑）
--------------------------------------------------------------------------
                         掃描率 km²/h        總 km        最長 km    覆蓋
  (1) rect 5x4 n=8    58.17 -> **58.89**  196.7->192.9  24.8->24.5  1.000000
  (2) four 1.25x4 n=8 48.36 -> **51.76**  236.5->221.0  29.8->27.8  1.000000
  (3) L 6.75 km² n=4  28.69 -> **29.25**   67.5-> 66.1  16.9->16.6  1.000000

  ⚠ 三個情境不足以下結論，所以另外掃了 **90 格**（10 種形狀 x n∈{4,8,12} x
    pitch/swath∈{0.5,0.7,0.9}）：v2b 勝 **80** / 負 10，掃描率變化中位
    **+5.02%**、平均 +5.81%、最差 −4.49%、最好 +23.50%；兩支的覆蓋率
    **90/90 都是 1.0000**。輸的 10 格集中在有洞/混合區的 n=12（群變小、
    格點對齊的好處被 depot 空程蓋過）。

  消融（掃描率，情境 1/2/3）
    完整版              58.89 / 51.76 / 29.25
    −列數規則(退回36列)  54.81 / 48.77 / 28.88
    −端點修剪 T         58.22 / 49.80 / 28.25
    −格點對齊           57.96 / 51.84 / 28.73
    −三者全退(≈v1 幾何)  57.90 / 48.18 / 27.30      ← 確認基準線被重現

  代價：時間 v1 的 2~4 倍（0.4~6 s vs 0.2~1.2 s），因為點數較多且多一道覆蓋驗證。

六、`MODE = "strips"`：可飛版（誠實標註它在這把尺上是輸的）
-----------------------------------------------------------
同一個檔還留了一條線段模型的路徑（列對齊 band x slice 分區 + 段級 2-opt/Or-opt），
輸出是長直線 + 180° 迴轉：情境(1) 156 航點 / 139 個 >60° 轉角，而點式版是
951 / 866、v1 是 1636 / 754。代價是掃描率 54.30 / 47.38 / 26.14 —— **比 v1 還差**。
原因量到了：線段模型不能免費走鋸齒邊界（換列要付 sqrt(s²+du²)，實測平均 255 m
vs 理想 113），而且每列必須一次走完 ⇒ 奇數列時一定在遠端收尾，depot 空程
19.2 km vs v1 的 9.8 km。**這把純幾何尺不獎勵可飛性**，所以預設是 "points"；
要做有轉彎成本的比較時再切到 "strips"。

⚠ 沒有解決的：點式版的航跡一樣不可飛（951 航點 / 866 個 >60° 轉角）。
⚠ 退化輸入：120x120 m 的小區 n=4 只會回 3 條航跡（點就那麼多），v1 回 4 條。
⚠ 順手抓到的一件事（不是這支的問題，是 v1 的）：`_cell_size` 的上限用
  `swath/sqrt(2)` = 114.31，是**真圓**的界；但 bench.metrics 用 quad_segs=8
  的內接 32 邊形，最小半徑只有 r·cos(π/32)，安全上限是 **113.76**。
  pitch/swath ≥ 0.808 時 v1 的 cell 會踩過去（見 `_rv2_claims.py` 第 3 節）。

只用 shapely + numpy + 標準庫（外加同目錄的 algo_grid_nn 當求解器）。
決定性：固定種子、排序全部顯式；同組輸入三次呼叫逐點相同（`_rv2_check.py`）。
"""
from __future__ import annotations

import math
import random

import numpy as np
from shapely.affinity import rotate as _rot
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

import algo_grid_nn as _v1      # 只借它的**求解器**（分群/再平衡/點式 2-opt），模型換掉

KEY = "grid_nn_v2b"

_SEED = 12345
_QS = 8                                   # 與 bench.metrics 的 buffer 同解析度
_R_FAC = math.cos(math.pi / (4 * _QS))    # quad_segs=8 內接 32 邊形的最小半徑比
_VERIFY_FAC = 0.999                       # 驗證時再保守一點
_SAFE = 0.995     # 設計半徑再留 0.5% 餘裕：不留的話幾何剛好卡在驗證半徑上
                  # ⇒ 每次都退回、每退一次重跑一整趟（實測 3.2s -> 28.4s）
_ATOM = 120.0                             # 分區粒度（m）；路由前會併回長直線
_WEIGHT_ITERS = 9
_REBAL_ROUNDS = 30
_ORO_MAX = 3
_EPS = 1e-7
_END_TRIM = True                          # 端點修剪（消融開關）
_ROW_RULE = True                          # 列數用「覆蓋需求」而非「鋪滿 bbox」
_USE_TILING = True                        # 列對齊分區（消融開關）
_USE_KMEANS = True                        # v1 式 2D 分群政策（消融開關）
_ALIGN = True     # 全域 u 格點對齊（消融開關）
MODE = "points"   # "points" = 尺上最好；"strips" = 可飛版；"both" = 兩個都跑取優


# ── 幾何 ──────────────────────────────────────────────────────────────
def _geoms(g):
    return [x for x in getattr(g, "geoms", [g]) if not x.is_empty]


def _min_width_angle(poly: Polygon) -> float:
    """最小寬度方向（rad）。條帶沿它走 ⇒ 列數最少。"""
    hull = poly.convex_hull
    if hull.geom_type != "Polygon":
        return 0.0
    cs = list(hull.exterior.coords)[:-1]
    best, best_w = 0.0, float("inf")
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        th = math.atan2(by - ay, bx - ax)
        c, s = math.cos(-th), math.sin(-th)
        # ⚠ 旋轉 −th 之後的 y 是 s·x + c·y。舊版 `-s*x + c*y` 因 s = −sin(th)
        #   而量成「鏡射後」的寬度（旋轉矩形實測多算 95%）。五個情境全軸對齊
        #   所以封存結果幾乎不受影響，但等寬邊的 argmin 取捨會變。
        ys = [(s * x + c * y) for x, y in cs]
        w = max(ys) - min(ys)
        if w < best_w:
            best_w, best = w, th
    return best


def _strip_segments(polys, pitch: float, swath: float, *, trim: bool,
                    with_s: bool = False):
    """條帶：[(part_key, row_idx, seg_j, u_a, u_b, world_a, world_b)]。

    `u` 是旋轉座標系裡沿列的位置（列對齊分區要用），world_* 是平面座標。
    """
    r = swath / 2.0 * _R_FAC
    out = []
    for pi, poly in enumerate(polys):
        for gi, g in enumerate(_geoms(poly)):
            if g.area <= 0:
                continue
            th = _min_width_angle(g)
            ca, sa = math.cos(th), math.sin(th)
            loc = _rot(g, -math.degrees(th), origin=(0.0, 0.0), use_radians=False)
            x0, y0, x1, y1 = loc.bounds
            H = y1 - y0
            if _ROW_RULE:
                if H <= 2.0 * r:
                    ys, s = [(y0 + y1) / 2.0], float("inf")
                else:
                    k = max(1, int(math.ceil((H - 2.0 * r) / pitch - 1e-9)))
                    s = (H - 2.0 * r) / k
                    ys = [y0 + r + i * s for i in range(k + 1)]
            else:                                   # 消融：v1 的鋪滿 bbox 規則
                ny = max(1, int(math.ceil(H / pitch)))
                oy = y0 - (ny * pitch - H) / 2.0
                s = pitch
                ys = [oy + (i + 0.5) * pitch for i in range(ny)]
            T = (math.sqrt(max(0.0, r * r - (s / 2.0) ** 2))
                 if (trim and math.isfinite(s)) else 0.0)
            last = len(ys) - 1
            for i, y in enumerate(ys):
                cut = T if 0 < i < last else 0.0
                cl = loc.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
                pieces = sorted((q for q in _geoms(cl)
                                 if q.geom_type == "LineString" and q.length > 1e-6),
                                key=lambda q: q.coords[0][0])
                for j, q in enumerate(pieces):
                    ua, ub = q.coords[0][0], q.coords[-1][0]
                    if ua > ub:
                        ua, ub = ub, ua
                    c = min(cut, 0.45 * (ub - ua))
                    ua, ub = ua + c, ub - c
                    rec = ((pi, gi), i, j, ua, ub,
                           (ca * ua - sa * y, sa * ua + ca * y),
                           (ca * ub - sa * y, sa * ub + ca * y))
                    out.append(rec + (s,) if with_s else rec)
    return out


def _row_points(polys, pitch: float, swath: float, *, trim: bool):
    """條帶幾何 -> **點式**必訪集合。v1 的求解器可以原封不動地吃它。

    這裡的關鍵是：點式模型的覆蓋保證不能靠「巡迴會把整列走成一條線」
    （我試過，rect n=8 覆蓋率掉到 99.99% —— 外側列與接縫處會漏），必須讓
    **光靠各點的刈幅圓**就蓋滿。於是三個尺寸互相牽制：

      邊界內縮 e、外側列取樣間距 d_out：sqrt((d_out/2)^2 + e^2) <= r
      列距 s、內側列取樣間距 d_in    ：sqrt((d_in /2)^2 + (s/2)^2) <= r
      內側列端點修剪 T               ：sqrt(T^2 + (s/2)^2) <= r

    v1 的做法等價於 e = s/2（用格中心），於是列數 = ceil(H/s) = 36。
    這裡改成**「能達到最少列數的最小 e」**：
        k = ceil((H - 2r)/pitch)          # 覆蓋所需的最少間隔數
        e = max(0, (H - k*pitch)/2)       # 還能維持 k 的最小內縮
    rect 5x4 km：k = 34 ⇒ **35 列**（v1 是 36 列），e = 77.0 m，
    d_out = 2*sqrt(r^2 - e^2)*0.98 = 45.6 m（外側兩列取樣加密，只多 ~200 點）。
    沿列走一列的長度跟取樣密度無關，所以加密不花錢。
    """
    r = swath / 2.0 * _R_FAC * _SAFE
    pts, dmin = [], float("inf")
    for pi, poly in enumerate(polys):
        for g in _geoms(poly):
            if g.area <= 0:
                continue
            th = _min_width_angle(g)
            ca, sa = math.cos(th), math.sin(th)
            loc = _rot(g, -math.degrees(th), origin=(0.0, 0.0), use_radians=False)
            x0, y0, x1, y1 = loc.bounds
            H = y1 - y0
            if H <= 2.0 * r:
                ys, s, e = [(y0 + y1) / 2.0], float("inf"), H / 2.0
            elif _ROW_RULE:
                k = max(1, int(math.ceil((H - 2.0 * r) / pitch - 1e-9)))
                e = max(0.0, min(r, (H - k * pitch) / 2.0))
                s = (H - 2.0 * e) / k
                ys = [y0 + e + i * s for i in range(k + 1)]
            else:                       # 消融：v1 的「用 cell 鋪滿 bbox」規則
                ny = max(1, int(math.ceil(H / pitch)))
                oy = y0 - (ny * pitch - H) / 2.0
                s, e = pitch, pitch / 2.0
                ys = [oy + (i + 0.5) * pitch for i in range(ny)]
            # ⚠ 外側列的取樣間距有**兩個**約束，我第一版只寫了前一個：
            #   (a) 對外：sqrt((d_out/2)^2 + e^2) <= r   —— 蓋到區域邊界
            #   (b) 對內：sqrt((d_out/2)^2 + (s/2)^2) <= r —— 蓋到與鄰列之間那一半
            #   漏掉 (b) 時 e < s/2 的組態（pitch/swath=0.9 的四塊區）會在外側列
            #   旁邊留下 11 個 ~300 m^2 的洞（缺口質心離最近點 88.9 m > 80.4）。
            lat = max(e, (s / 2.0) if math.isfinite(s) else e)
            d_out = max(8.0, 2.0 * math.sqrt(max(1e-6, r * r - lat * lat)))
            if math.isfinite(s):
                d_in = min(s, 2.0 * math.sqrt(max(1e-6, r * r - (s / 2.0) ** 2)))
                T = math.sqrt(max(0.0, r * r - (s / 2.0) ** 2)) if trim else 0.0
            else:
                d_in, T = d_out, 0.0
            last = len(ys) - 1
            for i, y in enumerate(ys):
                outer = (i == 0 or i == last)
                d, cut = (d_out, 0.0) if outer else (d_in, T)
                cl = loc.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
                for q in _geoms(cl):
                    if q.geom_type != "LineString" or q.length <= 1e-6:
                        continue
                    ua, ub = sorted((q.coords[0][0], q.coords[-1][0]))
                    c = min(cut, 0.45 * (ub - ua))
                    ua, ub = ua + c, ub - c
                    # ⚠ 取樣點必須落在**全域 u 格點**上，不能每列各自 L/ceil(L/d)。
                    #   各列自己均分 ⇒ 相鄰兩列的點在 u 上錯開 ⇒ 換列要付
                    #   sqrt(s^2+du^2) 而不是 s，v1 賴以生存的「走鋸齒不用錢」就沒了。
                    #   實測差 6.2 km / 情境(1)（193.92 vs 200.11）。
                    if _ALIGN:
                        j0 = int(math.ceil(ua / d - 1e-9))
                        j1 = int(math.floor(ub / d + 1e-9))
                        us = sorted({round(v, 6) for v in
                                     [ua] + [j * d for j in range(j0, j1 + 1)] + [ub]})
                    else:               # 消融：每列各自均分（會弄丟格點對齊）
                        kk = max(1, int(math.ceil((ub - ua) / max(d, 1e-6))))
                        us = [ua + (ub - ua) / kk * t for t in range(kk + 1)]
                    for t in range(1, len(us)):
                        dmin = min(dmin, us[t] - us[t - 1])
                    dmin = min(dmin, s if math.isfinite(s) else d)
                    for u in us:
                        pts.append((ca * u - sa * y, sa * u + ca * y))
    out, seen = [], set()
    for p in pts:
        key = (round(p[0], 6), round(p[1], 6))
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out, (dmin if math.isfinite(dmin) else pitch)


def _plan_points(polys, n_uav, swath_m, pitch_m, start, *, trim):
    """v1 的求解器 + v2b 的點集。兩種容量政策各跑一次取 makespan 小者（同 v1）。"""
    centers, cell = _row_points(polys, pitch_m, swath_m, trim=trim)
    if not centers:
        return []
    P = np.asarray(centers, float)
    best = None
    for dc in (True, False):
        groups = _v1._rebalance(_v1._cluster(P, n_uav, start, cell, depot_caps=dc),
                                centers, start, cell)
        tr, mk = [], 0.0
        for g in groups:
            if not g:
                continue
            seq, L = _v1._solve([tuple(start)] + [centers[i] for i in g])
            if len(seq) < 2:
                continue
            tr.append(_collinear(list(seq) + [tuple(start)]))
            mk = max(mk, L)
        if tr and (best is None or mk < best[0] - 1e-9):
            best = (mk, tr)
        if n_uav <= 1:
            break
    return best[1] if best else []


def _collinear(t, tol=1e-6):
    """拿掉共線同向的中間點：不改折線幾何、不改任何指標，只是航點數變少。"""
    if len(t) < 3:
        return t
    out = [t[0]]
    for i in range(1, len(t) - 1):
        ax, ay = t[i][0] - out[-1][0], t[i][1] - out[-1][1]
        bx, by = t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na > 1e-9 and nb > 1e-9 and ax * bx + ay * by > 0                 and abs(ax * by - ay * bx) <= tol * na * nb:
            continue
        out.append(t[i])
    out.append(t[-1])
    return out


def _atomize(segs, atom_max: float):
    """把條帶切成 ≤ atom_max 的原子（只為了讓分區有細粒度；路由前會併回去）。"""
    at = []
    for key, row, j, ua, ub, wa, wb in segs:
        L = math.dist(wa, wb)
        k = max(1, int(math.ceil(L / atom_max)))
        for t in range(k):
            f0, f1 = t / k, (t + 1) / k
            p0 = (wa[0] + (wb[0] - wa[0]) * f0, wa[1] + (wb[1] - wa[1]) * f0)
            p1 = (wa[0] + (wb[0] - wa[0]) * f1, wa[1] + (wb[1] - wa[1]) * f1)
            at.append(dict(part=key, row=row, seg=j, sub=t, a=p0, b=p1,
                           L=math.dist(p0, p1),
                           u=ua + (ub - ua) * (f0 + f1) / 2.0,
                           mid=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)))
    return at


def _merge(idxs, atoms):
    """同一條帶、下標連續的原子併回一條長直線 ⇒ 航點數不因切碎而變多。"""
    rows = {}
    for i in idxs:
        a = atoms[i]
        rows.setdefault((a["part"], a["row"], a["seg"]), []).append(i)
    out = []
    for key in sorted(rows.keys()):
        run = sorted(rows[key], key=lambda i: atoms[i]["sub"])
        cur = [run[0]]
        for i in run[1:]:
            if atoms[i]["sub"] == atoms[cur[-1]]["sub"] + 1:
                cur.append(i)
            else:
                out.append((atoms[cur[0]]["a"], atoms[cur[-1]]["b"], key))
                cur = [i]
        out.append((atoms[cur[0]]["a"], atoms[cur[-1]]["b"], key))
    return out


# ── 路由：段級 TSP（每段兩個方向）＋ 2-opt / Or-opt ────────────────────
def _endpoints(segs, idx, o):
    a, b = segs[idx][0], segs[idx][1]
    return (a, b) if o == 0 else (b, a)


def _conn(order, ori, segs, start):
    cur, c = start, 0.0
    for idx, o in zip(order, ori):
        pin, pout = _endpoints(segs, idx, o)
        c += math.dist(cur, pin)
        cur = pout
    return c + math.dist(cur, start)


def _nn_seed(segs, start):
    left, order, ori, cur = list(range(len(segs))), [], [], start
    while left:
        best = None
        for i in left:
            for o in (0, 1):
                pin, pout = _endpoints(segs, i, o)
                d = math.dist(cur, pin)
                if best is None or d < best[0] - 1e-12:
                    best = (d, i, o, pout)
        _, i, o, pout = best
        order.append(i)
        ori.append(o)
        left.remove(i)
        cur = pout
    return order, ori


def _orient(order, segs, start):
    ori, cur = [], start
    for i in order:
        a, b = segs[i][0], segs[i][1]
        o = 0 if math.dist(cur, a) <= math.dist(cur, b) else 1
        ori.append(o)
        cur = b if o == 0 else a
    return order, ori


def _serp_seed(segs, start):
    """依（part, 列序, 段序）走蛇形；方向交替由貪婪選擇決定。"""
    return _orient(sorted(range(len(segs)), key=lambda i: segs[i][2]), segs, start)


def _comb_seed(segs, start):
    """梳狀（隔列）起始解：出程走偶數列、回程走奇數列 ⇒ 終點回到出發那一側。

    ⚠ 這一條不是裝飾。列數為奇數時純蛇形一定在**遠端**收尾，rect n=8 的
    territory 實測要多付 3.15 km 的回家空程；梳狀只多付 (k−1)·s ≈ 0.9 km。
    2-opt 從蛇形起始解到不了梳狀（那是一次大重排），所以必須直接餵給它。
    """
    s = sorted(range(len(segs)), key=lambda i: segs[i][2])
    return _orient(s[0::2] + s[1::2][::-1], segs, start)


def _local(order, ori, segs, start):
    """2-opt + Or-opt。

    2-opt 在「段」上的形式：反轉一段順序**並翻轉其中每段的方向**。這個動作
    不改變被反轉區間內部的連接長度（d(O_k, I_{k+1}) 反過來仍是同一段距離），
    所以增益公式跟點式 2-opt 一樣只有四項。i == j 就是「單段翻向」。
    """
    m = len(order)
    if m == 0:
        return order, ori, 0.0
    pts = lambda k: _endpoints(segs, order[k], ori[k])       # noqa: E731
    improved, guard = True, 0
    while improved and guard < 100:
        improved, guard = False, guard + 1
        for i in range(m):                                   # --- 2-opt ---
            prev = start if i == 0 else pts(i - 1)[1]
            for j in range(i, m):
                nxt = start if j == m - 1 else pts(j + 1)[0]
                old = math.dist(prev, pts(i)[0]) + math.dist(pts(j)[1], nxt)
                new = math.dist(prev, pts(j)[1]) + math.dist(pts(i)[0], nxt)
                if old - new > _EPS:
                    order[i:j + 1] = order[i:j + 1][::-1]
                    ori[i:j + 1] = [1 - o for o in ori[i:j + 1]][::-1]
                    improved = True
        for L in range(1, _ORO_MAX + 1):                     # --- Or-opt ---
            i, bumped = 0, False
            while i + L <= m:
                prev = start if i == 0 else pts(i - 1)[1]
                nxt = start if i + L >= m else pts(i + L)[0]
                g_rem = (math.dist(prev, pts(i)[0])
                         + math.dist(pts(i + L - 1)[1], nxt) - math.dist(prev, nxt))
                if g_rem <= _EPS:
                    i += 1
                    continue
                seg_o, seg_r = order[i:i + L], ori[i:i + L]
                rest_o, rest_r = order[:i] + order[i + L:], ori[:i] + ori[i + L:]
                best = None
                for k in range(len(rest_o) + 1):
                    p = (start if k == 0
                         else _endpoints(segs, rest_o[k - 1], rest_r[k - 1])[1])
                    q = (start if k == len(rest_o)
                         else _endpoints(segs, rest_o[k], rest_r[k])[0])
                    base = math.dist(p, q)
                    for flip in (0, 1):
                        so = seg_o[::-1] if flip else seg_o
                        sr = [1 - o for o in seg_r][::-1] if flip else seg_r
                        head = _endpoints(segs, so[0], sr[0])[0]
                        tail = _endpoints(segs, so[-1], sr[-1])[1]
                        add = math.dist(p, head) + math.dist(tail, q) - base
                        if g_rem - add > _EPS and (best is None or add < best[0]):
                            best = (add, k, so, sr)
                if best is None:
                    i += 1
                    continue
                _, k, so, sr = best
                order[:] = rest_o[:k] + so + rest_o[k:]
                ori[:] = rest_r[:k] + sr + rest_r[k:]
                improved = bumped = True
                i = 0
            if bumped:
                break
    return order, ori, _conn(order, ori, segs, start)


def _route(idxs, atoms, start):
    """回傳 (折線, 總長度)。折線起訖都在 start。"""
    if not idxs:
        return [], 0.0
    segs = _merge(idxs, atoms)
    fixed = sum(math.dist(a, b) for a, b, _k in segs)
    best = None
    for seed in (_serp_seed, _comb_seed, _nn_seed):
        o, r = seed(segs, start)
        o, r, c = _local(list(o), list(r), segs, start)
        if best is None or c < best[0] - 1e-9:
            best = (c, list(o), list(r))
    conn, order, ori = best
    track = [tuple(start)]
    for i, oo in zip(order, ori):
        pin, pout = _endpoints(segs, i, oo)
        track.append(tuple(pin))
        track.append(tuple(pout))
    track.append(tuple(start))
    return track, fixed + conn


class _Router:
    """帶快取的路由器（分區搜尋會反覆對同一組原子求解）。"""

    def __init__(self, atoms, start):
        self.atoms, self.start, self.cache = atoms, start, {}

    def length(self, g):
        k = tuple(sorted(g))
        if k not in self.cache:
            self.cache[k] = _route(list(k), self.atoms, self.start)[1]
        return self.cache[k]


# ── 政策 1：列對齊分區（band × slice）＋ 切點權重迭代 ──────────────────
def _split_by_share(n_items, w_item, shares):
    """把已排序的 n_items 依累積權重切成 len(shares) 段，回傳 index 分組。"""
    tot = float(sum(w_item))
    k = len(shares)
    if tot <= 0 or k == 0:
        return [list(range(n_items))] + [[] for _ in range(max(0, k - 1))]
    ssum = float(sum(shares)) or 1.0
    tgt, cum = [], 0.0
    for s in shares[:-1]:
        cum += s / ssum
        tgt.append(cum * tot)
    out, cur, acc, t = [], [], 0.0, 0
    for i in range(n_items):
        cur.append(i)
        acc += w_item[i]
        while t < len(tgt) and acc >= tgt[t] - 1e-9:
            out.append(cur)
            cur, t = [], t + 1
    out.append(cur)
    while len(out) < k:
        out.append([])
    return out[:k]


def _largest_remainder(n: int, weights) -> list[int]:
    k = len(weights)
    if k > n:
        return []
    tot = float(sum(weights)) or 1.0
    raw = [n * w / tot for w in weights]
    base = [max(1, int(math.floor(v))) for v in raw]
    while sum(base) > n:
        j = int(np.argmax([base[i] - raw[i] if base[i] > 1 else -1e9
                           for i in range(k)]))
        if base[j] <= 1:
            return []
        base[j] -= 1
    while sum(base) < n:
        base[int(np.argmax([raw[i] - base[i] for i in range(k)]))] += 1
    return base


def _tile_groups(atoms, n, b, w):
    """b 個 band（連續列群）、band j 內 a_j 個 slice（沿 u 的直片）。"""
    rows = {}
    for i in range(len(atoms)):
        a = atoms[i]
        rows.setdefault((a["part"], a["row"]), []).append(i)
    keys = sorted(rows.keys())
    if len(keys) < b:
        return None
    rlen = [sum(atoms[i]["L"] for i in rows[k]) for k in keys]
    eq = _split_by_share(len(keys), rlen, [1.0] * b)          # 先等分決定 a_j
    if any(not g for g in eq):
        return None
    a_j = _largest_remainder(n, [sum(rlen[i] for i in g) for g in eq])
    if not a_j or sum(a_j) != n:
        return None
    off, bw = 0, []
    for j in range(b):
        bw.append(sum(w[off:off + a_j[j]]))
        off += a_j[j]
    bands = _split_by_share(len(keys), rlen, bw)
    if any(not g for g in bands):
        return None
    groups, off = [], 0
    for j, band in enumerate(bands):
        at = sorted((i for ki in band for i in rows[keys[ki]]),
                    key=lambda i: (atoms[i]["u"], atoms[i]["row"], atoms[i]["seg"]))
        sl = _split_by_share(len(at), [atoms[i]["L"] for i in at],
                             w[off:off + a_j[j]])
        groups += [[at[t] for t in piece] for piece in sl]
        off += a_j[j]
    return groups


def _tile_plan(atoms, n, router):
    """b 從 1 掃到 n，每個 b 做權重迭代。回傳 (makespan, groups) 或 None。"""
    best = None
    for b in range(1, n + 1):
        w = [1.0] * n
        for _ in range(_WEIGHT_ITERS):
            groups = _tile_groups(atoms, n, b, w)
            if groups is None:
                break
            lens = [router.length(g) if g else 0.0 for g in groups]
            mk = max(lens)
            if best is None or mk < best[0] - 1e-9:
                best = (mk, [list(g) for g in groups])
            mean = sum(lens) / len(lens)
            if mean <= 0:
                break
            w = [max(0.25, min(4.0, wi * (mean / max(li, 1e-6)) ** 0.5))
                 for wi, li in zip(w, lens)]
    return best


# ── 政策 2：v1 式容量受限 k-means（容量改以長度計）─────────────────────
def _kmeanspp(P, w, k, rng):
    n = len(P)
    idx = [rng.randrange(n)]
    d2 = ((P - P[idx[0]]) ** 2).sum(1) * w
    for _ in range(k - 1):
        tot = float(d2.sum())
        if tot <= 0:
            idx.append(rng.randrange(n))
        else:
            t, acc, pick = rng.random() * tot, 0.0, n - 1
            for j in range(n):
                acc += float(d2[j])
                if acc >= t:
                    pick = j
                    break
            idx.append(pick)
        d2 = np.minimum(d2, ((P - P[idx[-1]]) ** 2).sum(1) * w)
    return P[idx].copy()


def _assign(P, w, cent, caps):
    n, k = len(P), len(cent)
    D = np.sqrt(((P[:, None, :] - cent[None, :, :]) ** 2).sum(-1))
    pref = np.argsort(D, axis=1)
    srt = np.sort(D, axis=1)
    regret = (srt[:, 1] - srt[:, 0]) if k >= 2 else np.zeros(n)
    lab, used = np.full(n, -1, int), [0.0] * k
    for i in np.argsort(-regret, kind="stable"):
        i = int(i)
        for c in pref[i]:
            c = int(c)
            if used[c] + w[i] <= caps[c] + 1e-9:
                lab[i], used[c] = c, used[c] + w[i]
                break
        if lab[i] < 0:
            c = int(np.argmin([used[j] / max(caps[j], 1e-9) for j in range(k)]))
            lab[i], used[c] = c, used[c] + w[i]
    return lab


def _cluster(P, w, k, depot, *, depot_caps=True):
    n = len(P)
    if k <= 1 or n <= 1:
        return [list(range(n))]
    if n <= k:
        return [[i] for i in range(n)] + [[] for _ in range(k - n)]
    rng = random.Random(_SEED)
    cent, W, lab = _kmeanspp(P, w, k, rng), float(np.sum(w)), None
    for _ in range(20):
        base = W / k
        if depot_caps:
            d = np.hypot(cent[:, 0] - depot[0], cent[:, 1] - depot[1])
            caps = np.clip(base + 2.0 * (d.mean() - d),
                           0.55 * base, 1.6 * base).tolist()
        else:
            caps = [base] * k
        new = _assign(P, w, cent, caps)
        if lab is not None and np.array_equal(new, lab):
            break
        lab = new
        for c in range(k):
            m = lab == c
            if m.any():
                ww = w[m]
                cent[c] = (P[m] * ww[:, None]).sum(0) / max(float(ww.sum()), 1e-9)
    return [np.nonzero(lab == c)[0].tolist() for c in range(k)]


def _rebalance(groups, atoms, router):
    """以 makespan 為目標的實測爬山（v1 的做法，節點換成原子）。"""
    live = [list(g) for g in groups if g]
    if len(live) < 2:
        return live
    M = np.asarray([a["mid"] for a in atoms], float)
    lens = [router.length(g) for g in live]
    step = _ATOM
    for _ in range(_REBAL_ROUNDS):
        hi = int(np.argmax(lens))
        cent = {i: M[live[i]].mean(0) for i in range(len(live))}
        cands = sorted((i for i in range(len(live))
                        if i != hi and lens[i] < lens[hi] - 2.0 * step),
                       key=lambda i: lens[i])
        moved = False
        for lo in cands[:3]:
            k = max(1, min(len(live[hi]) - 1,
                           int((lens[hi] - lens[lo]) / (4.0 * step))))
            while k >= 1 and not moved:
                tgt = cent[lo]
                mv = set(sorted(live[hi],
                                key=lambda i: ((M[i][0] - tgt[0]) ** 2
                                               + (M[i][1] - tgt[1]) ** 2))[:k])
                nh = [i for i in live[hi] if i not in mv]
                if nh:
                    nl = live[lo] + sorted(mv)
                    lh, ll_ = router.length(nh), router.length(nl)
                    if max(lh, ll_) < lens[hi] - 1e-6:
                        live[hi], live[lo] = nh, nl
                        lens[hi], lens[lo] = lh, ll_
                        moved = True
                k //= 2
            if moved:
                break
        if not moved:
            break
    return live


# ── 組裝 ──────────────────────────────────────────────────────────────
def _tracks_of(groups, atoms, start):
    tr, mk = [], 0.0
    for g in groups:
        t, L = _route(g, atoms, start)
        if len(t) >= 2:
            tr.append(t)
            mk = max(mk, L)
    return mk, tr


def _build_strips(polys, n_uav, swath_m, pitch_m, start, *, trim):
    """線段模型（可飛版）：列對齊 tiling + v1 式 2D 分群，兩者取 makespan 小者。"""
    segs = _strip_segments(polys, pitch_m, swath_m, trim=trim)
    if not segs:
        return []
    atoms = _atomize(segs, _ATOM)
    if not atoms:
        return []
    router = _Router(atoms, start)
    cands = []
    if _USE_TILING:
        got = _tile_plan(atoms, n_uav, router)
        if got:
            cands.append(_tracks_of(got[1], atoms, start))
    if _USE_KMEANS and n_uav > 1:
        P = np.asarray([a["mid"] for a in atoms], float)
        w = np.asarray([max(a["L"], 1e-6) for a in atoms], float)
        for dc in (True, False):
            g = _rebalance(_cluster(P, w, n_uav, start, depot_caps=dc), atoms, router)
            cands.append(_tracks_of(g, atoms, start))
    if not cands:
        cands.append(_tracks_of([list(range(len(atoms)))], atoms, start))
    cands = [c for c in cands if c[1]]
    return min(cands, key=lambda c: c[0])[1] if cands else []


def _makespan(tracks):
    return max((sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t)))
                for t in tracks), default=float("inf"))


def _build(polys, n_uav, swath_m, pitch_m, start, *, trim):
    cands = []
    if MODE in ("points", "both"):
        t = _plan_points(polys, n_uav, swath_m, pitch_m, start, trim=trim)
        if t:
            cands.append(t)
    if MODE in ("strips", "both"):
        t = _build_strips(polys, n_uav, swath_m, pitch_m, start, trim=trim)
        if t:
            cands.append(t)
    return min(cands, key=_makespan) if cands else []


def _covers(polys, tracks, swath_m) -> bool:
    """比 bench.metrics 保守的覆蓋驗證（半徑 ×0.999、同樣 quad_segs=8）。"""
    if not tracks:
        return False
    area = unary_union(polys)
    swept = unary_union([LineString(t).buffer(swath_m / 2.0 * _VERIFY_FAC,
                                             quad_segs=_QS)
                         for t in tracks if len(t) >= 2])
    return area.difference(swept).area <= 1e-6 * max(area.area, 1.0)


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """條帶式覆蓋規劃：列對齊分區 + 段級 2-opt/Or-opt。

    回傳每機一條折線（平面座標、公尺），起點與終點都在 start。
    """
    n_uav = max(1, int(n_uav))
    swath_m, pitch_m = float(swath_m), float(pitch_m)
    start = (float(start[0]), float(start[1]))
    pitch = min(pitch_m, swath_m)
    for attempt in range(4):
        trims = (True, False) if (_END_TRIM and attempt == 0) else (False,)
        for trim in trims:
            tr = _build(polys, n_uav, swath_m, pitch, start, trim=trim)
            if tr and _covers(polys, tr, swath_m):
                return tr
        pitch *= 0.95
    return _build(polys, n_uav, swath_m, pitch, start, trim=False)
