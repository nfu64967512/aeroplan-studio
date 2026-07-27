#!/usr/bin/env python3
"""stot_score.py — 從 tot_node 的飛行記錄重算 STOT 命中 KPI(取代「最近取樣點」)。

為什麼需要這支:
  tot_node 回報的 `最近水平距` 是**所有取樣點裡最近的那一個**。俯衝地速約
  40-100 m/s,而分流線的 GLOBAL_POSITION_INT 實測只有 5-20 Hz ⇒ 相鄰取樣點
  相距 5-20 m。真正的最近通過點幾乎一定落在**兩個取樣點之間**,所以
  「最近取樣點距離」會系統性地**高估** miss —— 量測誤差和驗收門檻(5 m)同級。

  本工具改用「點到線段」的距離:機體在相鄰兩取樣點之間走直線(俯衝段本來就是
  直線),所以線段距離就是該區間真正的最近通過距離,量測量化誤差直接消掉。
  命中高度與命中時刻同樣在該點上做線性內插。

用法:
  python3 stot_score.py /tmp/tot_B1.csv [--log /tmp/trial_B1.log] [--json]
"""
import argparse
import csv
import json
import math
import os
import re
import sys

EARTH = 111320.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def seg_closest(px, py, ax, ay, bx, by):
    """點 P 到線段 AB 的最近距離,以及最近點在 AB 上的參數 t∈[0,1]。"""
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy), t


def load_zg(log_path):
    """從 tot_node log 撈每機自量的紅心地表高度(相對 HOME)。"""
    zg = {}
    if not log_path or not os.path.exists(log_path):
        return zg
    pat = re.compile(r"UAV(\d+) 紅心地表 z_g = ([+-]?[\d.]+)")
    with open(log_path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = pat.search(ln)
            if m:
                zg[int(m.group(1))] = float(m.group(2))
    return zg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--log", default=None, help="tot_node log(讀 z_g)")
    ap.add_argument("--geom", default=None, help="*_geom.csv(讀目標座標)")
    ap.add_argument("--ground", type=float, default=None,
                    help="無 log 時的統一地表高度 fallback")
    ap.add_argument("--phase", default="DIVE", help="只算這個階段(預設 DIVE)")
    ap.add_argument("--alt-max", type=float, default=30.0,
                    help="命中高度門檻(m);沿用 _kpi_parse.py 的 ALT_MAX 預設 30")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    geom = a.geom or a.csv.replace(".csv", "_geom.csv")
    T = None
    if os.path.exists(geom):
        with open(geom) as f:
            for r in csv.DictReader(f):
                if r["kind"] == "target":
                    T = (float(r["lat"]), float(r["lon"]))
    if T is None:
        print("找不到目標座標(%s)" % geom, file=sys.stderr)
        return 2

    zg = load_zg(a.log)
    tracks = {}
    with open(a.csv) as f:
        for r in csv.DictReader(f):
            if r["phase"] != a.phase:
                continue
            i = int(r["sysid"])
            tracks.setdefault(i, []).append(
                (float(r["t"]), float(r["lat"]), float(r["lon"]),
                 float(r["alt"]), float(r["gs"] or 0)))

    ml = mlon(T[0])
    res = {}
    for i in sorted(tracks):
        pts = tracks[i]
        best = None
        for k in range(len(pts) - 1):
            t0, la0, lo0, z0, g0 = pts[k]
            t1, la1, lo1, z1, g1 = pts[k + 1]
            ax, ay = (la0 - T[0]) * EARTH, (lo0 - T[1]) * ml
            bx, by = (la1 - T[0]) * EARTH, (lo1 - T[1]) * ml
            # 取樣間跳太遠(> 400 m)代表中間斷訊,直線假設不成立,跳過
            if math.hypot(bx - ax, by - ay) > 400.0:
                continue
            dmin, tt = seg_closest(0.0, 0.0, ax, ay, bx, by)
            if best is None or dmin < best[0]:
                z = z0 + tt * (z1 - z0)
                tm = t0 + tt * (t1 - t0)
                best = (dmin, z, tm, g0 + tt * (g1 - g0))
        if best is None:
            continue
        g = zg.get(i, a.ground)
        # ── 真正的命中點:軌跡與地面的交點 ────────────────────────────────
        # ⚠️ 「最近通過距離」會美化結果:機體若在目標前 200 m 觸地再貼地滑進紅心,
        #    最近距離看起來很小,但那不是命中紅心 —— 它是在 200 m 外墜地的。
        #    物理上正確的命中點 = 軌跡穿過 AGL=0 的那一點。沒穿過就是從空中飛過。
        touch = None
        if g is not None:
            # ⚠️ 要取「離紅心最近的那個」穿越點,不是第一個。AGL 是相對**紅心地表**
            #    算的,所以機體在遠處飛過較高地形時可能先出現一次 AGL<=0 的穿越,
            #    那不是命中。實測 B21 的 UAV2:最近通過 0.72 m、最低 AGL 0.42 m
            #    (其實很準),卻因為取到 2554 m 外那個早期穿越,被判成誤差 2554 m、
            #    散布 37.9 s —— 一趟好的航次被誤判成失敗。
            for k in range(len(pts) - 1):
                a0 = pts[k][3] - g
                a1 = pts[k + 1][3] - g
                if a0 > 0.0 >= a1:
                    f = a0 / (a0 - a1) if (a0 - a1) != 0 else 0.0
                    la = pts[k][1] + f * (pts[k + 1][1] - pts[k][1])
                    lo = pts[k][2] + f * (pts[k + 1][2] - pts[k][2])
                    tm = pts[k][0] + f * (pts[k + 1][0] - pts[k][0])
                    dd = math.hypot((la - T[0]) * EARTH, (lo - T[1]) * ml)
                    if touch is None or dd < touch[0]:
                        touch = (dd, tm)
        # 俯衝階段地速(驗收主項:要夠快)。峰值與觸地瞬間都報 ——
        # 峰值通常出現在觸地前,兩者一起看才知道「衝下來的過程」有多快。
        gs_all = [p[4] for p in pts if p[4] > 0]
        res[i] = {
            "gs_peak_ms": round(max(gs_all), 1) if gs_all else None,
            "miss_m": round(best[0], 2),
            "alt_agl_m": round(best[1] - g, 2) if g is not None else None,
            "alt_rel_m": round(best[1], 2),
            "t_s": round(best[2], 2),
            "gs_ms": round(best[3], 1),
            "impact_miss_m": round(touch[0], 2) if touch else None,
            "impact_t_s": round(touch[1], 2) if touch else None,
            "n_samples": len(pts),
            "z_g": g,
        }

    if not res:
        print("沒有 %s 階段的資料" % a.phase, file=sys.stderr)
        return 2

    misses = [v["miss_m"] for v in res.values()]
    times = [v["t_s"] for v in res.values()]
    alts = [abs(v["alt_agl_m"]) for v in res.values()
            if v["alt_agl_m"] is not None]
    summary = {
        "n": len(res),
        "max_miss_m": round(max(misses), 2),
        "spread_s": round(max(times) - min(times), 2),
        "max_abs_alt_m": round(max(alts), 2) if alts else None,
        "per_uav": res,
    }
    if a.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0

    print("目標 %.6f/%.6f    階段=%s" % (T[0], T[1], a.phase))
    print("%-5s %9s %9s %11s %9s | %12s %9s"
          % ("UAV", "gs峰值", "gs觸地", "最低AGL(m)", "最近(m)",
             "觸地誤差(m)", "觸地t(s)"))
    for i in sorted(res):
        v = res[i]
        print("%-5d %9s %9.1f %11s %9.2f | %12s %9s"
              % (i, "n/a" if v["gs_peak_ms"] is None else "%.1f" % v["gs_peak_ms"],
                 v["gs_ms"],
                 "n/a" if v["alt_agl_m"] is None else "%.2f" % v["alt_agl_m"],
                 v["miss_m"],
                 "未觸地" if v["impact_miss_m"] is None
                 else "%.2f" % v["impact_miss_m"],
                 "-" if v["impact_t_s"] is None else "%.2f" % v["impact_t_s"]))
    print("─" * 78)
    touched = {i: v for i, v in res.items() if v["impact_miss_m"] is not None}
    if touched:
        tm = [v["impact_miss_m"] for v in touched.values()]
        tt = [v["impact_t_s"] for v in touched.values()]
        print("★ 觸地基準(軌跡穿過 AGL=0 的那一點,%d/%d 架真的落地):"
              % (len(touched), len(res)))
        print("   命中誤差 max = %.2f m;觸地時間散布 = %.2f s"
              % (max(tm), (max(tt) - min(tt)) if len(tt) > 1 else 0.0))
    else:
        print("★ 觸地基準:六機都沒有真的落地(全部從空中飛過)")
    print("─" * 78)
    # ⭐ 使用者 2026-07-28 重排優先序:俯衝階段地速要夠快(100 m/s)最優先、
    #    散布 ≤3 s 次之、精度再次之。所以速度放最前面報。
    gsp = [v["gs_peak_ms"] for v in res.values() if v["gs_peak_ms"] is not None]
    if gsp:
        print("★ 俯衝階段地速:六機峰值 %.1f – %.1f m/s(最慢那架 %.1f)  目標 100"
              % (min(gsp), max(gsp), min(gsp)))
    print("六機最大水平誤差 = %.2f m   (門檻 5,優先序最低)" % summary["max_miss_m"])
    if summary["max_abs_alt_m"] is not None:
        print("六機最大命中高度 = %.2f m   (門檻 %.0f)"
              % (summary["max_abs_alt_m"], a.alt_max))
    print("命中時間散布     = %.2f s   (門檻 3.0)" % summary["spread_s"])
    # ⚠️ 判定必須含命中高度。先前漏掉這一項,結果水平誤差和散布達標時就印「通過」,
    #    而那一趟六機其實是從紅心上空 35-175 m 飛過 —— 三項要全過才算過。
    miss_ok = summary["max_miss_m"] <= 5.0
    spread_ok = summary["spread_s"] <= 3.0
    alt_ok = (summary["max_abs_alt_m"] is not None
              and summary["max_abs_alt_m"] <= a.alt_max)
    ok = miss_ok and spread_ok and alt_ok and len(res) == 6
    fails = [n for n, good in (("水平誤差", miss_ok), ("散布", spread_ok),
                               ("命中高度", alt_ok), ("六機到齊", len(res) == 6))
             if not good]
    print("判定:%s" % ("✅ 三項全過" if ok else "❌ 未達標(%s)" % "、".join(fails)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
