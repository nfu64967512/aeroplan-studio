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
        res[i] = {
            "miss_m": round(best[0], 2),
            "alt_agl_m": round(best[1] - g, 2) if g is not None else None,
            "alt_rel_m": round(best[1], 2),
            "t_s": round(best[2], 2),
            "gs_ms": round(best[3], 1),
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
    print("%-5s %10s %12s %10s %9s %8s %8s"
          % ("UAV", "miss(m)", "alt_AGL(m)", "alt_rel", "t(s)", "gs", "n"))
    for i in sorted(res):
        v = res[i]
        print("%-5d %10.2f %12s %10.2f %9.2f %8.1f %8d"
              % (i, v["miss_m"],
                 "n/a" if v["alt_agl_m"] is None else "%.2f" % v["alt_agl_m"],
                 v["alt_rel_m"], v["t_s"], v["gs_ms"], v["n_samples"]))
    print("─" * 70)
    print("六機最大水平誤差 = %.2f m   (門檻 5)" % summary["max_miss_m"])
    if summary["max_abs_alt_m"] is not None:
        print("六機最大命中高度 = %.2f m   (門檻 ~0)" % summary["max_abs_alt_m"])
    print("命中時間散布     = %.2f s   (門檻 3.0)" % summary["spread_s"])
    ok = (summary["max_miss_m"] <= 5.0 and summary["spread_s"] <= 3.0
          and len(res) == 6)
    print("判定:%s" % ("✅ 通過" if ok else "❌ 未達標"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
