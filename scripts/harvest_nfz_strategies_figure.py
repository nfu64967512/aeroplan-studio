"""禁航區四種處理策略 + 崩潰門檻 + HV4SITL7 實飛航跡。

    python scripts/harvest_nfz_strategies_figure.py [--pdf]

四格
----
(a) **HV4SITL7 實飛**：4/4 落地，實飛航跡一公尺都沒進禁航區。
(b) **崩潰門檻**：禁航區蓋住某一塊作業區的比例 vs 上工台數。
    蓋住 ≥42.9% 時整趟任務歸零，而且**連不相干的區也一起被丟掉**。
(c) **原區 + 不把禁航區交給規劃器**：航跡事後繞行後不進禁航區，但相機仍拍進去。
(d) **切區 + 不交 + 事後繞轉場**：先用 BCD 把區切開繞過禁航區，相機闖入減半。

⚠ (a) 之外的三格都是**規劃層**的結果，不是實飛。
⚠ 「原區 + 交禁航區給規劃器」與「切區 + 交禁航區」兩種都回**空計畫**（0 台），
   畫不出來，所以沒有它們的格子 —— 那正是本圖要說的事。
"""
from __future__ import annotations

import argparse
import csv
import json
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

import logging  # noqa: E402
logging.disable(logging.CRITICAL)

from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape  # noqa: E402
from mission.swarm_coordinator import (SwarmCoordinator,  # noqa: E402
                                       split_areas_around_nfz)
from scripts.hv_sitl_scenarios import HOME, _grid, fleet_of, rect  # noqa: E402

M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
ALT, HFOV = 140.0, 60.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
C_TRK, C_NFZ, C_AREA = "#e8590c", "#e03131", "#343a40"

AREAS = _grid(4, 260, 300, 1500)


def xy(lat, lon):
    return ((lon - HOME[1]) * M_LON, (lat - HOME[0]) * M_LAT)


def _zone(v):
    return FenceZone(name="NFZ1", category=ZoneCategory.NFZ, shape=ZoneShape.POLYGON,
                     vertices=v, alt_min_m=0.0, alt_max_m=500.0)


def plan(areas, zones, split=False):
    """回傳每機航跡（平面座標）。空計畫回 []。"""
    fleet = fleet_of(1, 3)
    cls = {i + 1: c for i, (_n, c, _s, _v, _r) in enumerate(fleet)}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0}
              for i, (n, _c, _s, v, r) in enumerate(fleet)]
    c = SwarmCoordinator()
    c.use_hv_dccpp = True
    c.hv_split_areas_around_nfz = split
    c.hv_class_of = lambda d: cls[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_alternates = [(23.972571, 120.90)]
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = zones
    c.hv_params = {"hfov_deg": HFOV, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                   "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0,
                   "hv_terrain_clearance_m": 36.0}
    try:
        r = c.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=150.0,
                                  overlap_rate=0.30, coverage_altitude=ALT,
                                  mounting_angle_deg=0.0)
    except Exception:                                       # noqa: BLE001
        return []
    ap = r.get("assembled_paths") or {}
    return [[xy(w.lat, w.lon) for w in p.waypoints] for p in ap.values()
            if len(getattr(p, "waypoints", [])) >= 2]


def draw(ax, tracks, nfz_xy, areas_xy, title, note):
    for p in areas_xy:
        ax.plot(*p.exterior.xy, "--", color=C_AREA, lw=1.0, alpha=0.85)
    for z in nfz_xy:
        ax.fill(*z.exterior.xy, color=C_NFZ, alpha=0.30, lw=0)
        ax.plot(*z.exterior.xy, color=C_NFZ, lw=1.5)
    if tracks:
        swept = unary_union([LineString(t).buffer(SWATH / 2, quad_segs=8)
                             for t in tracks if len(t) >= 2])
        for g in getattr(swept, "geoms", [swept]):
            ax.fill(*g.exterior.xy, color="#adb5bd", alpha=0.35, lw=0)
        for t in tracks:
            ax.plot([q[0] for q in t], [q[1] for q in t], color=C_TRK, lw=0.7, alpha=0.9)
    ax.set_title(title, fontsize=9.5)
    ax.set_aspect("equal", "datalim")
    ax.grid(alpha=0.22, lw=0.5)
    ax.tick_params(labelsize=7)
    ax.text(0.015, 0.985, note, transform=ax.transAxes, va="top", fontsize=7.6,
            family="monospace",
            bbox=dict(fc="white", ec="0.7", alpha=0.93, boxstyle="round,pad=0.32"))


def stats(tracks, nfz_xy, areas_xy):
    if not tracks:
        return dict(n=0, km=0.0, cov=0.0, inside=0.0, imaged=0.0)
    Z = unary_union(nfz_xy)
    A = unary_union(areas_xy)
    swept = unary_union([LineString(t).buffer(SWATH / 2, quad_segs=8) for t in tracks])
    return dict(n=len(tracks),
                km=sum(LineString(t).length for t in tracks) / 1000.0,
                cov=A.intersection(swept).area / A.area,
                inside=sum(LineString(t).intersection(Z).length for t in tracks),
                imaged=swept.intersection(Z).area)


def main() -> int:
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--pdf", action="store_true")
    args = ap_.parse_args()

    areas_xy = [Polygon([xy(la, lo) for la, lo in a["polygon"]]) for a in AREAS]
    fig, axes = plt.subplots(2, 2, figsize=(15.0, 11.0))
    a1, a2, a3, a4 = axes.ravel()

    # ── (a) HV4SITL7 實飛 ────────────────────────────────────────
    out = ROOT / "runs" / "artifacts" / "HV4SITL7"
    meta = json.loads((out / "scenario.json").read_text(encoding="utf-8"))
    nfz7 = [Polygon([xy(la, lo) for la, lo in v]) for v in meta["nfz"]]
    flown = []
    for u in ("A1", "B2", "B3", "B4"):
        rows = [r for r in csv.DictReader((out / u / "flight.csv").open(encoding="utf-8"))
                if r["lat"]]
        t = [xy(float(r["lat"]), float(r["lon"])) for r in rows]
        t = [p for i, p in enumerate(t) if i == 0 or p != t[i - 1]]
        if len(t) >= 2:
            flown.append(t)
    s = stats(flown, nfz7, areas_xy)
    draw(a1, flown, nfz7, areas_xy,
         "(a) HV4SITL7 flown on SITL, NFZ on a transit leg",
         f"flown   {s['n']} UAV  {s['km']:6.2f} km  coverage {s['cov'] * 100:5.2f} %\n"
         f"track inside NFZ  {s['inside']:6.1f} m\n"
         f"imaged inside NFZ {s['imaged']:8.1f} m$^2$")

    # ── (b) 崩潰門檻 ─────────────────────────────────────────────
    fracs, nplanes = [], []
    z2 = Polygon([xy(la, lo) for la, lo in AREAS[1]["polygon"]])
    for he, hn in ((25, 30), (40, 45), (60, 70), (90, 110), (105, 130),
                   (120, 150), (135, 175), (150, 200), (200, 260)):
        v = rect(-750, 0, he, hn)
        zp = Polygon([xy(la, lo) for la, lo in v])
        fracs.append(zp.intersection(z2).area / z2.area * 100)
        nplanes.append(len(plan(AREAS, [_zone(v)])))
    a2.plot(fracs, nplanes, "o-", color=C_TRK, lw=1.8, ms=6)
    a2.axhline(4, color="0.6", ls=":", lw=0.8)
    a2.set_xlabel("share of survey area Z2 blocked by the NFZ [%]", fontsize=8.5)
    a2.set_ylabel("aircraft actually given work", fontsize=8.5)
    a2.set_title("(b) one blocked area takes the whole mission down\n"
                 "the other three areas are 1.5 km away and untouched", fontsize=9.5)
    a2.set_ylim(-0.4, 4.4)
    a2.grid(alpha=0.25, lw=0.5)
    a2.tick_params(labelsize=8)
    a2.annotate("whole mission empty:\nall 4 areas dropped",
                xy=(fracs[-1], 0), xytext=(fracs[len(fracs) // 2], 1.6), fontsize=8.4,
                color=C_NFZ, arrowprops=dict(arrowstyle="->", color=C_NFZ, lw=1.0))

    # ── (c)(d) 兩種可行策略 ──────────────────────────────────────
    big = rect(-750, 0, 150, 200)                       # 蓋住 Z2 的 63.4%
    zb = [Polygon([xy(la, lo) for la, lo in big])]
    for ax, (areas, split, title) in zip(
            (a3, a4),
            ((AREAS, False, "(c) original areas, NFZ withheld from the planner"),
             (split_areas_around_nfz(AREAS, [_zone(big)], inset_m=SWATH / 2), False,
              "(d) areas split around the NFZ (BCD), NFZ withheld"))):
        tr = plan(areas, [], split=split)
        if tr:
            # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
            sys.path.insert(0, str(ROOT / "research" / "cpp_benchmark"))
            from bench import nfz_correct
            tr, added, _rr = nfz_correct(tr, zb)
        st = stats(tr, zb, areas_xy)
        draw(ax, tr, zb, areas_xy, title,
             f"planned {st['n']} UAV  {st['km']:6.2f} km  coverage {st['cov'] * 100:5.1f} %\n"
             f"track inside NFZ  {st['inside']:6.1f} m  (after reroute)\n"
             f"imaged inside NFZ {st['imaged']:8.0f} m$^2$")

    fig.legend(handles=[
        Patch(fc=C_NFZ, alpha=0.30, ec=C_NFZ, label="no-fly zone"),
        Patch(fc="#adb5bd", alpha=0.35, label="swept by the camera"),
        Line2D([], [], color=C_TRK, lw=1.2, label="HARVEST track"),
        Line2D([], [], color=C_AREA, ls="--", lw=1.0, label="survey area"),
    ], loc="lower center", ncol=4, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.suptitle("HARVEST and no-fly zones\n"
                 "camera swath 161.7 m at 140 m AGL; panels (c)(d) are planned, "
                 "panel (a) is flown on SITL", fontsize=12)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))

    d = ROOT / "exports"
    d.mkdir(exist_ok=True)
    fig.savefig(d / "harvest_nfz_strategies.png", dpi=170)
    print("寫出", d / "harvest_nfz_strategies.png")
    if args.pdf:
        fig.savefig(d / "harvest_nfz_strategies.pdf")
        print("寫出", d / "harvest_nfz_strategies.pdf")
    print("\n門檻曲線：")
    for f, n in zip(fracs, nplanes):
        print(f"  Z2 被蓋住 {f:5.1f}%  → 上工 {n} 台")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
