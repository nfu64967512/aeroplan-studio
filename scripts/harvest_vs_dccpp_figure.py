"""HARVEST vs 舊 DCCPP 疊圖：同一組輸入、同一把尺，畫在同一張座標上。

    python scripts/harvest_vs_dccpp_figure.py            # 產生 exports/harvest_vs_dccpp.png
    python scripts/harvest_vs_dccpp_figure.py --pdf      # 另外輸出向量檔（投稿用）

圖上畫什麼
----------
每一格是**一個組態**，兩個規劃器疊在同一組座標上：

* 黑色虛線     作業區邊界
* 填色         掃過的區域（航跡緩衝相機刈幅 / 2）
    - 灰   兩邊都掃到
    - 藍   只有 DCCPP 掃到
    - 橘   只有 HARVEST 掃到
    - 紅   **兩邊都沒掃到**（作業區內）← 這格才是重點
* 細線         各自的航跡（藍 = DCCPP，橘 = HARVEST）

量法的三條規矩（踩過才知道）
--------------------------
1. **航跡取全部航點，不做航段篩選。** 兩個規劃器的 segment 標籤慣例不同：
   舊版把條帶標成 OPERATION，HARVEST 只有 4–7 個點是 OPERATION、其餘標成
   TRANSFER，而且末端 10–15 個 LANDING 點還在作業區裡。篩選會讓 HARVEST 的
   覆蓋率被量成 0–96%，而同一份計畫用全航點量是 100%。
2. **距離用航點折線，不用 `total_length_m`。** 後者是一本帳：
   `core/dccpp/dccpp_path_builder.py:255` 把轉場簡化成只吐終點，卻照樣把
   解析 Dubins 弧長記進去。實測舊版 1A+3B 帳面 43.36 km、折線只有 38.13 km。
3. **兩邊用同一個物理刈幅** `2·h·tan(HFOV/2)`，因為要問的是「同一台相機飛
   這個圖案蓋不蓋得到」，不是各自宣稱的條帶間距。

自檢
----
第一格是 HV4SITL3 真的飛過的幾何與參數，它匯出的航點與實飛航跡**都是 100%**。
本腳本量 HARVEST 那一格若不是 100%，就是量法又壞了，會直接 assert 失敗。
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
logging.disable(logging.CRITICAL)

from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
PERF = {"A": (21.5, 200.0), "B": (19.0, 120.0)}

C_OLD, C_NEW = "#1c6fd6", "#e8590c"
C_BOTH, C_NONE = "#c9ccd1", "#e03131"


def _xy(lat: float, lon: float) -> tuple[float, float]:
    return ((lon - HOME[1]) * _M_LON, (lat - HOME[0]) * _M_LAT)


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def rect(cx: float, cy: float, he: float, hn: float):
    return [_ll(cx - he, cy - hn), _ll(cx + he, cy - hn),
            _ll(cx + he, cy + hn), _ll(cx - he, cy + hn)]


def _drones(n_a: int, n_b: int, width: float):
    fleet = [("A", *PERF["A"])] * n_a + [("B", *PERF["B"])] * n_b
    return ([{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME, "speed": v,
              "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
              "fov_width": width} for i, (c, v, r) in enumerate(fleet)],
            {i + 1: c for i, (c, _v, _r) in enumerate(fleet)})


def plan(use_hv: bool, areas, n_a, n_b, *, alt, width, overlap, hfov=60.0, vfov=45.0,
         wind=(5.0, 270.0)):
    """回傳 (每機航跡[平面座標], 折線總長 km, 分到工作的台數)。"""
    drones, cls = _drones(n_a, n_b, width)
    c = SwarmCoordinator()
    if use_hv:
        c.use_hv_dccpp = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME
        c.hv_wind = wind
        c.hv_fence_zones = []
        c.hv_params = {"hfov_deg": hfov, "vfov_deg": vfov, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    try:
        res = c.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=width,
                                    overlap_rate=overlap, coverage_altitude=alt,
                                    mounting_angle_deg=0.0)
    except Exception:                                    # noqa: BLE001
        return [], 0.0, 0
    tracks, km = [], 0.0
    for p in (res.get("assembled_paths") or {}).values():
        pts = [_xy(w.lat, w.lon) for w in getattr(p, "waypoints", [])]
        pts = [q for i, q in enumerate(pts) if i == 0 or q != pts[i - 1]]
        if len(pts) < 2:
            continue
        tracks.append(pts)
        km += sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
                  for i in range(1, len(pts))) / 1000.0
    return tracks, km, len(tracks)


def swept(tracks, swath_m: float):
    bufs = [LineString(t).buffer(swath_m / 2.0, quad_segs=8) for t in tracks if len(t) >= 2]
    return unary_union(bufs) if bufs else None


def coverage(area_polys, sw):
    if sw is None:
        return 0.0
    tot = sum(p.area for p in area_polys)
    return sum(p.intersection(sw).area for p in area_polys) / tot if tot else 0.0


# ══════════════════════════════════════════════════════════════════════
#  組態
# ══════════════════════════════════════════════════════════════════════
def _grid4():
    return [{"area_id": i + 1, "priority": 1.0, "polygon": rect(-2250 + 1500 * i, 0, 260, 300)}
            for i in range(4)]


SCENARIOS = [
    dict(key="a", title="(a) Flown geometry, 1 Pioneer + 3 HERO\n"
                        "panel convention: pitch 113.2 m, overlap_rate 0",
         areas=_grid4(), n_a=1, n_b=3, alt=140.0, width=113.16, overlap=0.0,
         note="both complete; this is the configuration flown in SITL",
         expect_hv_full=True),
    dict(key="b", title="(b) Single 5.0 x 4.0 km area, 8 Pioneer\n"
                        "parameters page convention: pitch 150 m, overlap_rate 0.30",
         areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 2500, 2000)}],
         n_a=8, n_b=0, alt=140.0, width=150.0, overlap=0.30,
         note="DCCPP drops strips at the far edge as overlap_rate rises"),
    dict(key="c", title="(c) Single 5.0 x 4.0 km area, 4 Pioneer + 4 HERO\n"
                        "panel convention: pitch 113.2 m, overlap_rate 0",
         areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 2500, 2000)}],
         n_a=4, n_b=4, alt=140.0, width=113.16, overlap=0.0,
         note="HARVEST assigns no aircraft on a single-cell area with a mixed fleet"),
    dict(key="d", title="(d) Same 20 km$^2$ split into 4 areas, 4 Pioneer + 4 HERO\n"
                        "panel convention: pitch 113.2 m, overlap_rate 0",
         areas=[{"area_id": i + 1, "priority": 1.0,
                 "polygon": rect(-1875 + 1250 * i, 0, 625, 2000)} for i in range(4)],
         n_a=4, n_b=4, alt=140.0, width=113.16, overlap=0.0,
         note="splitting the same area rescues HARVEST: 0 -> 8 aircraft"),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdf", action="store_true", help="另外輸出向量檔")
    args = ap.parse_args()

    fig, axes = plt.subplots(2, 2, figsize=(15.0, 11.0))
    rows = []
    for ax, sc in zip(axes.ravel(), SCENARIOS):
        swath = 2.0 * sc["alt"] * math.tan(math.radians(60.0) / 2.0)
        kw = dict(alt=sc["alt"], width=sc["width"], overlap=sc["overlap"])
        t_old, km_old, n_old = plan(False, sc["areas"], sc["n_a"], sc["n_b"], **kw)
        t_new, km_new, n_new = plan(True, sc["areas"], sc["n_a"], sc["n_b"], **kw)
        s_old, s_new = swept(t_old, swath), swept(t_new, swath)
        polys = []
        for a in sc["areas"]:
            p = Polygon([_xy(la, lo) for la, lo in a["polygon"]])
            polys.append(p if p.is_valid else p.buffer(0))
        cov_old, cov_new = coverage(polys, s_old), coverage(polys, s_new)

        area = unary_union(polys)
        both = area.intersection(s_old).intersection(s_new) if (s_old and s_new) else None
        only_o = area.intersection(s_old).difference(s_new) if s_old else None
        only_n = area.intersection(s_new).difference(s_old) if s_new else None
        none = area
        for s in (s_old, s_new):
            if s is not None:
                none = none.difference(s)

        def fill(geom, color, alpha, label=None):
            if geom is None or geom.is_empty:
                return
            for g in getattr(geom, "geoms", [geom]):
                if g.is_empty or g.geom_type != "Polygon":
                    continue
                ax.fill(*g.exterior.xy, color=color, alpha=alpha, lw=0, label=label)
                label = None

        fill(none, C_NONE, 0.55)
        fill(both, C_BOTH, 0.75)
        fill(only_o, C_OLD, 0.28)
        fill(only_n, C_NEW, 0.28)
        for p in polys:
            ax.plot(*p.exterior.xy, "k--", lw=1.1, alpha=0.85)
        for t in t_old:
            ax.plot([q[0] for q in t], [q[1] for q in t], color=C_OLD, lw=0.7, alpha=0.9)
        for t in t_new:
            ax.plot([q[0] for q in t], [q[1] for q in t], color=C_NEW, lw=0.7, alpha=0.9)

        ax.set_title(sc["title"], fontsize=9.5)
        ax.set_aspect("equal", "datalim")
        ax.grid(alpha=0.22, lw=0.5)
        ax.set_xlabel("east [m]", fontsize=8)
        ax.set_ylabel("north [m]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.text(0.015, 0.985,
                f"DCCPP    {n_old} UAV   {km_old:6.1f} km   coverage {cov_old * 100:5.1f} %\n"
                f"HARVEST  {n_new} UAV   {km_new:6.1f} km   coverage {cov_new * 100:5.1f} %\n"
                f"{sc['note']}",
                transform=ax.transAxes, va="top", ha="left", fontsize=7.6,
                family="monospace",
                bbox=dict(fc="white", ec="0.7", alpha=0.92, boxstyle="round,pad=0.35"))
        rows.append((sc["key"], n_old, km_old, cov_old, n_new, km_new, cov_new))
        if sc.get("expect_hv_full"):
            # 自檢：這一格是真的飛過的組態，匯出與實飛都量到 100%
            assert cov_new > 0.999, (
                f"自檢失敗：已知答案是 100%，量出 {cov_new * 100:.1f}% ⇒ 量法壞了")

    fig.legend(handles=[
        Patch(fc=C_BOTH, alpha=0.75, label="swept by both"),
        Patch(fc=C_OLD, alpha=0.28, label="swept by DCCPP only"),
        Patch(fc=C_NEW, alpha=0.28, label="swept by HARVEST only"),
        Patch(fc=C_NONE, alpha=0.55, label="swept by neither"),
        Line2D([], [], color=C_OLD, lw=1.2, label="DCCPP track"),
        Line2D([], [], color=C_NEW, lw=1.2, label="HARVEST track"),
        Line2D([], [], color="k", ls="--", lw=1.1, label="survey area"),
    ], loc="lower center", ncol=7, fontsize=8.5, frameon=False,
        bbox_to_anchor=(0.5, 0.005))
    fig.suptitle("HARVEST vs DCCPP on identical inputs\n"
                 "coverage judged geometrically: full waypoint track buffered by the "
                 "camera swath 2h tan(HFOV/2) = 161.7 m at 140 m AGL",
                 fontsize=11.5)
    fig.tight_layout(rect=(0, 0.045, 1, 0.945))

    out = ROOT / "exports"
    out.mkdir(exist_ok=True)
    png = out / "harvest_vs_dccpp.png"
    fig.savefig(png, dpi=170)
    print("寫出", png)
    if args.pdf:
        pdf = out / "harvest_vs_dccpp.pdf"
        fig.savefig(pdf)
        print("寫出", pdf)

    print(f"\n{'格':>3s}{'DCCPP 台':>9s}{'km':>8s}{'覆蓋':>8s}"
          f"{'HARVEST 台':>11s}{'km':>8s}{'覆蓋':>8s}")
    for k, n_o, km_o, c_o, n_n, km_n, c_n in rows:
        print(f"{k:>3s}{n_o:9d}{km_o:8.1f}{c_o * 100:7.1f}%{n_n:11d}{km_n:8.1f}{c_n * 100:7.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
